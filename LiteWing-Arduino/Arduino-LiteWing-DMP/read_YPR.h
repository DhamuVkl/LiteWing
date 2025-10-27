#ifndef READ_YPR_H
#define READ_YPR_H

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// Pin definitions for ESP32-S3
#define SDA_PIN 11
#define SCL_PIN 10
#define INTERRUPT_PIN 12
#define LED_PIN 9

// Configuration constants
#define CALIBRATION_SAMPLES 1000
#define CALIBRATION_DELAY_MS 3
#define STABILIZATION_TIME_MS 6000
#define STABILITY_THRESHOLD 0.75
#define STABILITY_SAMPLES 10
#define LED_BLINK_INTERVAL 500
#define USE_HARDCODED_OFFSETS 1

// Hardcoded calibration offsets (replace with your values)
int16_t gyroXOffset = 220;
int16_t gyroYOffset = 76;
int16_t gyroZOffset = -85;
int16_t accelXOffset = -1000;
int16_t accelYOffset = 500;
int16_t accelZOffset = 1200;

MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
float currentYaw = 0, currentPitch = 0, currentRoll = 0;
float lastYaw = 0, lastPitch = 0, lastRoll = 0;
float yawRate = 0, pitchRate = 0, rollRate = 0;
float yawOffset = 0, pitchOffset = 0, rollOffset = 0;
unsigned long lastTime = 0;
bool isStable = false;
int stableCount = 0;
unsigned long lastBlinkTime = 0;
bool ledState = false;

float Roll_Rate, Pitch_Rate, Yaw_Rate;
float Kalman_Roll_Angle = 0;
float Kalman_Pitch_Angle = 0;
float Kalman_Yaw_Angle = 0;

volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

void readMPUData();
void printMPUValues();
void calibrateMPU();
void setupMPU();
void updateAngles();
void setZeroOffsets();

void calibrateMPU() {
    Serial.println(F("Calibrating MPU6050... Keep sensor stationary."));
    int32_t gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
    int32_t accelXSum = 0, accelYSum = 0, accelZSum = 0;
    unsigned long startTime = millis();
    
    for (int i = 0; i < CALIBRATION_SAMPLES && millis() - startTime < 10000; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        gyroXSum += gx;
        gyroYSum += gy;
        gyroZSum += gz;
        accelXSum += ax;
        accelYSum += ay;
        accelZSum += az;
        delay(CALIBRATION_DELAY_MS);
    }
    
    if (millis() - startTime >= 10000) {
        Serial.println(F("Calibration timed out."));
    }
    
    gyroXOffset = gyroXSum / CALIBRATION_SAMPLES;
    gyroYOffset = gyroYSum / CALIBRATION_SAMPLES;
    gyroZOffset = gyroZSum / CALIBRATION_SAMPLES;
    accelXOffset = accelXSum / CALIBRATION_SAMPLES;
    accelYOffset = accelYSum / CALIBRATION_SAMPLES;
    accelZOffset = (accelZSum / CALIBRATION_SAMPLES) - 16384;
    
    Serial.println(F("Calibration complete. Update hardcoded offsets:"));
    Serial.print(F("int16_t gyroXOffset = ")); Serial.print(gyroXOffset); Serial.println(F(";"));
    Serial.print(F("int16_t gyroYOffset = ")); Serial.print(gyroYOffset); Serial.println(F(";"));
    Serial.print(F("int16_t gyroZOffset = ")); Serial.print(gyroZOffset); Serial.println(F(";"));
    Serial.print(F("int16_t accelXOffset = ")); Serial.print(accelXOffset); Serial.println(F(";"));
    Serial.print(F("int16_t accelYOffset = ")); Serial.print(accelYOffset); Serial.println(F(";"));
    Serial.print(F("int16_t accelZOffset = ")); Serial.print(accelZOffset); Serial.println(F(";"));
}

void setupMPU() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    
    Serial.println(F("Initializing MPU6050..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    if (!mpu.testConnection()) {
        Serial.println(F("MPU6050 connection failed."));
        while (1);
    }
    Serial.println(F("MPU6050 connection successful"));

    #if USE_HARDCODED_OFFSETS
        Serial.println(F("Using hardcoded offsets:"));
        Serial.print(F("Gyro (X,Y,Z): ")); Serial.print(gyroXOffset); Serial.print(F(", "));
        Serial.print(gyroYOffset); Serial.print(F(", ")); Serial.println(gyroZOffset);
        Serial.print(F("Accel (X,Y,Z): ")); Serial.print(accelXOffset); Serial.print(F(", "));
        Serial.print(accelYOffset); Serial.print(F(", ")); Serial.println(accelZOffset);
        mpu.setXGyroOffset(gyroXOffset);
        mpu.setYGyroOffset(gyroYOffset);
        mpu.setZGyroOffset(gyroZOffset);
        mpu.setXAccelOffset(accelXOffset);
        mpu.setYAccelOffset(accelYOffset);
        mpu.setZAccelOffset(accelZOffset);
    #else
        calibrateMPU();
        mpu.setXGyroOffset(gyroXOffset);
        mpu.setYGyroOffset(gyroYOffset);
        mpu.setZGyroOffset(gyroZOffset);
        mpu.setXAccelOffset(accelXOffset);
        mpu.setYAccelOffset(accelYOffset);
        mpu.setZAccelOffset(accelZOffset);
    #endif

    Serial.println(F("Initializing DMP..."));
    if (mpu.dmpInitialize() != 0) {
        Serial.println(F("DMP Initialization failed."));
        while (1);
    }

    Serial.println(F("Waiting for DMP to stabilize..."));
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    for (int i = 0; i < 10; i++) {
        while (!mpuInterrupt && fifoCount < packetSize) {
            fifoCount = mpu.getFIFOCount();
        }
        readMPUData();
        delay(10);
    }

    for (int i = 0; i < 2; i++) {
        delay(STABILIZATION_TIME_MS / 2);
        mpu.resetFIFO();
        Serial.println(F("FIFO reset."));
    }
    Serial.println(F("DMP stabilized."));
    lastTime = millis();
}

void printMPUValues() {
    Serial.print(currentYaw); Serial.print("\t");
    Serial.print(currentPitch); Serial.print("\t");
    Serial.print(currentRoll); Serial.print("\t");
    Serial.print(yawRate); Serial.print("\t");
    Serial.print(pitchRate); Serial.print("\t");
    Serial.println(rollRate);
}

void setZeroOffsets() {
    Serial.println(F("Orientation stable. Zeroing yaw, pitch, roll..."));
    int validSamples = 0;
    yawOffset = 0;
    pitchOffset = 0;
    rollOffset = 0;

    mpu.resetFIFO();
    delay(50);

    for (int i = 0; i < 30 && validSamples < 15; i++) {
        while (!mpuInterrupt && fifoCount < packetSize) {
            fifoCount = mpu.getFIFOCount();
        }
        readMPUData();
        if (ypr[0] == 0 && ypr[1] == 0 && ypr[2] == 0) continue;

        yawOffset += ypr[0] * 180/M_PI;
        pitchOffset += ypr[1] * 180/M_PI;
        rollOffset += ypr[2] * 180/M_PI;
        validSamples++;
        delay(10);
    }

    if (validSamples >= 10) {
        yawOffset /= validSamples;
        pitchOffset /= validSamples;
        rollOffset /= validSamples;
        Serial.print(F("Zero offsets - Yaw: ")); Serial.print(yawOffset);
        Serial.print(F(", Pitch: ")); Serial.print(pitchOffset);
        Serial.print(F(", Roll: ")); Serial.println(rollOffset);
        digitalWrite(LED_PIN, HIGH);
        isStable = true;
        Serial.println("Yaw\tPitch\tRoll\tYawRate\tPitchRate\tRollRate");
    } else {
        Serial.println(F("Failed to calculate valid offsets."));
        yawOffset = 0;
        pitchOffset = 0;
        rollOffset = 0;
    }
}

void readMPUData() {
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow! FIFO reset."));
        return;
    }
    if (!(mpuIntStatus & 0x02)) return;

    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    float norm = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm < 0.9 || norm > 1.1) return;

    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;

    currentYaw = ypr[0] * 180/M_PI - yawOffset;
    currentPitch = ypr[1] * 180/M_PI - pitchOffset;
    currentRoll = ypr[2] * 180/M_PI - rollOffset;

    float alpha = 0.99;
    currentYaw = alpha * currentYaw + (1 - alpha) * lastYaw;
    currentPitch = alpha * currentPitch + (1 - alpha) * lastPitch;
    currentRoll = alpha * currentRoll + (1 - alpha) * lastRoll;

    yawRate = (currentYaw - lastYaw) / deltaTime;
    pitchRate = (currentPitch - lastPitch) / deltaTime;
    rollRate = (currentRoll - lastRoll) / deltaTime;

    lastYaw = currentYaw;
    lastPitch = currentPitch;
    lastRoll = currentRoll;
    lastTime = currentTime;
}

void updateAngles() {
    if (!dmpReady) return;

    if (!isStable) {
        unsigned long currentTime = millis();
        if (currentTime - lastBlinkTime >= LED_BLINK_INTERVAL) {
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState ? HIGH : LOW);
            lastBlinkTime = currentTime;
        }
    }

    if (mpuInterrupt || fifoCount >= packetSize) {
        readMPUData();

        if (!isStable) {
            if (abs(yawRate) < STABILITY_THRESHOLD && abs(pitchRate) < STABILITY_THRESHOLD && abs(rollRate) < STABILITY_THRESHOLD) {
                stableCount++;
                if (stableCount >= STABILITY_SAMPLES) {
                    setZeroOffsets();
                }
            } else {
                stableCount = 0;
            }
        }

        if (isStable) {
            Kalman_Roll_Angle = currentPitch;
            Kalman_Pitch_Angle = currentRoll;
            Kalman_Yaw_Angle = currentYaw;
            Roll_Rate = pitchRate;
            Pitch_Rate = rollRate;
            Yaw_Rate = yawRate;

            // Uncomment to print values for debugging
            // printMPUValues();
        }
    }

    // Ensure loop runs at ~100Hz to match DMP packet rate
    //delay(8); // Approximate delay to achieve 100Hz (10ms per cycle)
}

#endif // READ_YPR_H