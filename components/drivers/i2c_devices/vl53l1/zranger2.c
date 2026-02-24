/**
*    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 * 
 * ESP-Drone Firmware
 * 
 * Copyright 2019-2020  Espressif Systems (Shanghai) 
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * vl53l1x.c: Time-of-flight distance sensor driver
 */


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "range.h"
#include "i2cdev.h"
#include "zranger2.h"
#include "vl53l1x.h"
#include "cf_math.h"
#define DEBUG_MODULE "ZR2"
#include "debug_cf.h"

// Measurement noise model (defaults for SHORT mode)
static float expPointA = 2.5f;
static float expStdA = 0.0025f; // STD at elevation expPointA [m]
static float expPointB = 4.0f;
static float expStdB = 0.2f;    // STD at elevation expPointB [m]
static float expCoeff;

// Noise model constants per distance mode
#define SHORT_EXP_POINT_A  2.5f
#define SHORT_EXP_STD_A    0.0025f
#define SHORT_EXP_POINT_B  4.0f
#define SHORT_EXP_STD_B    0.2f

#define LONG_EXP_POINT_A   2.5f
#define LONG_EXP_STD_A     0.005f    // Higher base noise in long mode
#define LONG_EXP_POINT_B   4.0f
#define LONG_EXP_STD_B     0.4f      // More noise at distance in long mode

#define RANGE_OUTLIER_LIMIT 5000 // the measured range is in [mm]

// Auto distance mode switching thresholds (hysteresis to prevent oscillation)
#define SHORT_TO_LONG_THRESHOLD  1150  // Switch to LONG above 1.15m (mm)
#define LONG_TO_SHORT_THRESHOLD   950  // Switch to SHORT below 0.95m (mm)

// Timing budgets per mode
#define SHORT_MODE_TIMING_BUDGET_US  25000
#define SHORT_MODE_PERIOD_MS         25
#define LONG_MODE_TIMING_BUDGET_US   140000   // 140ms — ST's recommended max for full 4m range
#define LONG_MODE_PERIOD_MS          145      // Must be >= timing budget + 4ms (per ST docs)

static int16_t range_last = 0;

static bool isInit;

static VL53L1_Dev_t dev;
static VL53L1_DistanceModes currentDistanceMode = VL53L1_DISTANCEMODE_SHORT;
static uint32_t currentTaskPeriodMs = SHORT_MODE_PERIOD_MS;
static bool modeSwitchPending = false;

/**
 * Recalculate the noise model coefficient from current expStd/expPoint values.
 */
static void updateNoiseModel(void)
{
  expCoeff = logf(expStdB / expStdA) / (expPointB - expPointA);
}

/**
 * Switch the sensor distance mode at runtime.
 * Stops measurement, reconfigures, restarts with settling delay.
 */
static void switchDistanceMode(VL53L1_DistanceModes newMode)
{
  if (newMode == currentDistanceMode)
    return;

  VL53L1_StopMeasurement(&dev);
  vTaskDelay(M2T(10));  // Let sensor fully stop

  if (newMode == VL53L1_DISTANCEMODE_SHORT) {
    VL53L1_SetDistanceMode(&dev, VL53L1_DISTANCEMODE_SHORT);
    VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev, SHORT_MODE_TIMING_BUDGET_US);
    VL53L1_SetInterMeasurementPeriodMilliSeconds(&dev, SHORT_MODE_PERIOD_MS);
    currentTaskPeriodMs = SHORT_MODE_PERIOD_MS;

    // Update noise model for short mode
    expPointA = SHORT_EXP_POINT_A;
    expStdA   = SHORT_EXP_STD_A;
    expPointB = SHORT_EXP_POINT_B;
    expStdB   = SHORT_EXP_STD_B;

    DEBUG_PRINTI("ZR2: Switched to SHORT mode\n");
  } else {
    VL53L1_SetDistanceMode(&dev, VL53L1_DISTANCEMODE_LONG);
    VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev, LONG_MODE_TIMING_BUDGET_US);
    VL53L1_SetInterMeasurementPeriodMilliSeconds(&dev, LONG_MODE_PERIOD_MS);
    currentTaskPeriodMs = LONG_MODE_PERIOD_MS;

    // Update noise model for long mode
    expPointA = LONG_EXP_POINT_A;
    expStdA   = LONG_EXP_STD_A;
    expPointB = LONG_EXP_POINT_B;
    expStdB   = LONG_EXP_STD_B;

    DEBUG_PRINTI("ZR2: Switched to LONG mode\n");
  }

  updateNoiseModel();
  currentDistanceMode = newMode;

  VL53L1_StartMeasurement(&dev);

  // Wait for sensor to settle and produce first valid measurement in new mode
  vTaskDelay(M2T(currentTaskPeriodMs * 2));

  // Clear any stale interrupt from previous mode and get fresh start
  VL53L1_clear_interrupt_and_enable_next_range(&dev, VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK);

  // Signal that we just switched — skip first reading
  modeSwitchPending = true;
}

static uint16_t zRanger2GetMeasurement(VL53L1_Dev_t *dev)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;
    VL53L1_RangingMeasurementData_t rangingData;
    uint8_t dataReady = 0;
    uint16_t range;

    status = VL53L1_GetMeasurementDataReady(dev, &dataReady);
    if (status != VL53L1_ERROR_NONE || dataReady == 0)
    {
        return RANGE_OUTLIER_LIMIT + 1; // Not ready or error
    }

    status = VL53L1_GetRangingMeasurementData(dev, &rangingData);

    // Mode-specific range status filtering:
    // SHORT mode: strict — only accept RANGE_VALID (0)
    // LONG mode:  also accept NO_WRAP_CHECK_FAIL (6) — wrap-around is
    //             impossible at drone flight altitudes (1-3m)
    bool rangeValid = false;
    if (status == VL53L1_ERROR_NONE) {
        if (rangingData.RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID) {
            rangeValid = true;
        } else if (currentDistanceMode == VL53L1_DISTANCEMODE_LONG &&
                   rangingData.RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL) {
            rangeValid = true;
        }
    }

    if (rangeValid) {
        range = rangingData.RangeMilliMeter;
    } else {
        range = RANGE_OUTLIER_LIMIT + 1;
    }

    // Handshake: Clear interrupt to allow the sensor to start the next internal measurement immediately
    VL53L1_clear_interrupt_and_enable_next_range(dev, VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK);

    return range;
}

void zRanger2Init(void)
{
  if (isInit)
    return;

  if (vl53l1xInit(&dev, I2C1_DEV))
  {
    DEBUG_PRINTI("Z-down sensor [OK]\n");
  }
  else
  {
    DEBUG_PRINTW("Z-down sensor [FAIL]\n");
    return;
  }

  xTaskCreate(zRanger2Task, ZRANGER2_TASK_NAME, ZRANGER2_TASK_STACKSIZE, NULL, ZRANGER2_TASK_PRI, NULL);

  // pre-compute constant in the measurement noise model for kalman
  updateNoiseModel();

  isInit = true;
}

bool zRanger2Test(void)
{
  if (!isInit)
    return false;

  return true;
}

void zRanger2Task(void* arg)
{
  TickType_t lastWakeTime;

  systemWaitStart();

  // Configure sensor for continuous (back-to-back) mode — start in SHORT
  VL53L1_StopMeasurement(&dev);
  VL53L1_SetDistanceMode(&dev, VL53L1_DISTANCEMODE_SHORT);
  VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev, SHORT_MODE_TIMING_BUDGET_US);
  VL53L1_SetInterMeasurementPeriodMilliSeconds(&dev, SHORT_MODE_PERIOD_MS);
  currentDistanceMode = VL53L1_DISTANCEMODE_SHORT;
  currentTaskPeriodMs = SHORT_MODE_PERIOD_MS;

  // Start continuous ranging
  VL53L1_StartMeasurement(&dev);

  // Wait for first measurement to be ready
  vTaskDelay(M2T(SHORT_MODE_PERIOD_MS * 2));

  static int consecutive_errors = 0;
  lastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&lastWakeTime, M2T(currentTaskPeriodMs));

    // After a mode switch, skip first read and reset timing
    if (modeSwitchPending) {
      modeSwitchPending = false;
      consecutive_errors = 0;
      lastWakeTime = xTaskGetTickCount();  // Reset timing baseline
      continue;
    }

    uint16_t range_new = zRanger2GetMeasurement(&dev);

    // Only update and push to estimator if range is valid and within limits
    if (range_new < RANGE_OUTLIER_LIMIT) {
      consecutive_errors = 0;
      range_last = range_new;
      rangeSet(rangeDown, range_last / 1000.0f);
      
      float distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
      float stdDev = expStdA * (1.0f  + expf( expCoeff * (distance - expPointA)));
      rangeEnqueueDownRangeInEstimator(distance, stdDev, xTaskGetTickCount());

      // Auto distance mode switching with hysteresis
      if (currentDistanceMode == VL53L1_DISTANCEMODE_SHORT &&
          range_new > SHORT_TO_LONG_THRESHOLD) {
        switchDistanceMode(VL53L1_DISTANCEMODE_LONG);
      } else if (currentDistanceMode == VL53L1_DISTANCEMODE_LONG &&
                 range_new < LONG_TO_SHORT_THRESHOLD) {
        switchDistanceMode(VL53L1_DISTANCEMODE_SHORT);
      }
    } else {
      consecutive_errors++;
      // If sensor is stuck for > 1 second, force a restart
      if (consecutive_errors > (int)(1000 / currentTaskPeriodMs)) {
        DEBUG_PRINTW("ZR2: Sensor stuck, restarting acquisition...\n");
        VL53L1_StopMeasurement(&dev);
        vTaskDelay(M2T(50));
        VL53L1_StartMeasurement(&dev);
        vTaskDelay(M2T(currentTaskPeriodMs * 2));
        VL53L1_clear_interrupt_and_enable_next_range(&dev, VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK);
        consecutive_errors = 0;
        lastWakeTime = xTaskGetTickCount();
      }
    }
  }

}

static uint8_t disable = 0;
#define PARAM_CORE (1<<5)
#define PARAM_PERSISTENT (1 << 8)
#define PARAM_ADD_CORE(TYPE, NAME, ADDRESS) \
  PARAM_ADD(TYPE | PARAM_CORE, NAME, ADDRESS)

PARAM_GROUP_START(deck)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger2, &isInit)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcACS37800, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcActiveMarker, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcAI, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcBigQuad, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcCPPM, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, cpxOverUART2, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcFlapperDeck, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcGTGPS, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLedRing, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcLhTester, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLighthouse4, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcLoadcell, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcDWM1000, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLoco, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcMultiranger, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcOA, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcServo, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcUSD, &disable)
PARAM_GROUP_STOP(deck)


static uint32_t effect = 0;
static uint32_t neffect = 0;
PARAM_GROUP_START(ring)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_PERSISTENT, effect, &effect)
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, neffect, &neffect)
PARAM_GROUP_STOP(ring)
