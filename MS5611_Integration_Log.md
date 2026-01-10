# MS5611 Barometric Sensor Integration Log

## Objective
Enable the MS5611 barometric pressure sensor in the LiteWing firmware and visualize the data (Pressure, Temperature, ASL) using a Python script.

## Timeline of Issues & Fixes

### 1. Connection Failure
**Issue:**
The firmware reported `[FAIL]` for the MS5611 I2C connection upon startup.
```
W (2797) SENSORS: MS5611 I2C connection [FAIL].
```

**Root Cause:**
The initialization logic in `sensors_mpu6050_hm5883L_ms5611.c` contained a placeholder `if (false)` block, forcing the initialization check to always fail regardless of the actual sensor status.

**Fix:**
Updated the initialization check to use the actual return value of the driver function.
```c
// Before
if (false) { ... }

// After
if (ms5611Init(I2C0_DEV)) { ... }
```

### 2. Zero Data Readings
**Issue:**
After fixing the connection check, the sensor was detected (`[OK]`), but the Python script received all zeros for pressure, temperature, and ASL.

**Root Cause:**
The firmware was configured to read the MS5611 via the MPU6050's "Slave Mode" (auto-read). However, the MS5611 requires a specific state machine (Start Conversion -> Wait -> Read) which the MPU6050's simple register read loop cannot handle. This resulted in reading empty or invalid data buffers.

**Fix:**
Refactored `sensors_mpu6050_hm5883L_ms5611.c` to:
1.  **Disable MPU6050 Slave Config:** Removed the code that set up the MPU6050 to read the MS5611.
2.  **Implement Direct Read:** Modified `sensorsTask` to call `ms5611GetData()` directly. This function handles the correct start/wait/read sequence for the MS5611.
3.  **Adjust Buffer Calculation:** Corrected the `dataLen` calculation to remove the MS5611 size contribution since it is no longer read via the MPU6050 buffer.

### 3. Logic Explanation (MS5611 vs LPS25H)
**Context:**
The code structure previously referenced LPS25H logic, which uses simple linear scaling.

**Clarification Added:**
Added code comments explaining why MS5611 uses a different approach:
*   **LPS25H:** Uses simple linear scaling (Value / Sensitivity).
*   **MS5611:** Uses complex polynomial compensation involving 6 factory calibration coefficients. It requires calculating a temperature delta (`dT`) first to dynamically correct pressure offset and sensitivity. This results in higher resolution (10cm) and better temperature stability.

## Current Status
*   **Firmware:** Fully configured to initialize and read MS5611 directly.
*   **Python Script:** `MS5611.py` created to plot real-time Pressure, Temperature, and Altitude Above Sea Level (ASL).
*   **Hardware:** Sensor is detected and communicating over I2C.
