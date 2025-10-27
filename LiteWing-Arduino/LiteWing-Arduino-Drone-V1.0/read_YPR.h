#ifndef READ_YPR_H
#define READ_YPR_H

// Make the MPU object accessible to other files
extern MPU6050 mpu;
extern float loop_time;
extern float Desired_Yaw_Rate;

// Pre-calculated constants for faster math
const float INV_4096 = 1.0f / 4096.0f;
const float INV_65_5 = 1.0f / 65.5f;
const float MY_RAD_TO_DEG = 57.295779513f;
const float DT = 0.004f;

// MPU calibration values
float Acc_Error_X = 0.0354f;
float Acc_Error_Y = 0.0192f;
float Acc_Error_Z = 0.0629f;
float Gyro_Error_X = -2.9746f;
float Gyro_Error_Y = 0.6573f;
float Gyro_Error_Z = -2.8568f;

// Dynamic bias estimation variables
float Gyro_Dynamic_Bias_Z = 0;
float Gyro_Noise_Threshold = 0.2f;

float yaw_correction_factor = 0.2f;

// Angle and rate variables
float Acc_Roll_Angle, Acc_Pitch_Angle;
float Roll_Rate, Pitch_Rate, Yaw_Rate;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float Yaw_Angle = 0;

// Moving average filter - optimized for speed
const int GYRO_FILTER_SIZE = 3; // Reduced to 3 samples for less lag
const float INV_GYRO_FILTER_SIZE = 1.0f / GYRO_FILTER_SIZE;
float GyroZ_history[GYRO_FILTER_SIZE] = {0};
int gyro_history_index = 0;
float gyro_filter_sum = 0; // Store sum for faster moving average

// Ultra-responsive complementary filter variable
float alpha = 0.1f;

// Kalman variables
float Kalman_Roll_Angle = 0;
float Kalman_Pitch_Angle = 0;
float Kalman_Yaw_Angle = 0, Kalman_Yaw_Angle_Uncertinity = 2*2;
float Kalman_Output[] = {0, 0};
uint32_t LoopTimer;

// Stability detection variables
bool isStable = false;
unsigned long lastStableTime = 0;
unsigned long lastMovementTime = 0;
int stable_reading_count = 0;
const int REQUIRED_STABLE_READINGS = 500;

// Temperature compensation - can be disabled for speed
bool enable_temp_comp = false; // Set to false for max speed
float temp_coefficient = 0.002f;
float baseline_temp = 25.0f;
float current_temp = 25.0f;

// Drift correction variables
float yaw_drift_correction = 0;
bool drift_correction_enabled = false;
unsigned long drift_detection_start = 0;
const unsigned long DRIFT_DETECTION_TIME = 5000000; // 5 seconds in microseconds

// Slew rate limiter variables
float last_roll = 0, last_pitch = 0;
float max_change_per_cycle = 0.9f; // Slightly increased for more responsiveness

// Function prototypes
void readMPUData();
void updateRollPitchAngles();
void printMPUValues();
void calibrateMPU();
void setupMPU();
void updateAngles();
float applyMovingAverageFilter(float newValue);
float estimateGyroBias(float gyroReading);

// Improved Kalman filter for yaw that better handles different bias directions
void kalman_1d_yaw(float &KalmanState, float &KalmanUncertainty, float KalmanInput) {
  // Improved threshold filter with symmetric behavior
  if (abs(KalmanInput) < Gyro_Noise_Threshold) {
    KalmanInput = 0;
  }
  
  // Fast Kalman prediction
  KalmanState += DT * KalmanInput;
  KalmanUncertainty += DT * DT * 2.5f * 2.5f;
  
  // Directional drift correction - correctly handles both positive and negative drift
  if (drift_correction_enabled && isStable && (int32_t)(micros() - lastMovementTime) > DRIFT_DETECTION_TIME) {
    // Apply a tiny force towards zero instead of a percentage reduction
    // This works equally well for positive and negative KalmanState values
    if (abs(KalmanState) > 0.01f) {
      // Pull back towards zero with a tiny constant force
      float correction_step = 0.001f;
      if (KalmanState > 0) {
        KalmanState -= correction_step;
      } else {
        KalmanState += correction_step;
      }
    } else {
      // Very close to zero, just set to zero to avoid oscillation
      KalmanState = 0;
    }
  }
  
  Kalman_Output[0] = KalmanState; 
  Kalman_Output[1] = KalmanUncertainty;
}

// Optimized roll/pitch update
void updateRollPitchAngles() {
  // Fast magnitude calculation
  float accel_magnitude = sqrt(AccX*AccX + AccY*AccY + AccZ*AccZ);
  
  // Simple alpha selection
  alpha = 0.1f; // Default
  
  if (abs(accel_magnitude - 1.0f) > 0.1f) {
    alpha = 0.01f;
  }
  
  if (Acc_Roll_Angle * Kalman_Roll_Angle < 0 && abs(Kalman_Roll_Angle) > 5.0f) {
    alpha = 0.001f;
  }
  
  // Fast complementary filter
  float gyro_term_roll = Kalman_Roll_Angle + Roll_Rate * DT;
  float gyro_term_pitch = Kalman_Pitch_Angle + Pitch_Rate * DT;
  
  Kalman_Roll_Angle = (1.0f - alpha) * gyro_term_roll + alpha * Acc_Roll_Angle;
  Kalman_Pitch_Angle = (1.0f - alpha) * gyro_term_pitch + alpha * Acc_Pitch_Angle;
  
  // Apply Kalman for yaw
  kalman_1d_yaw(Kalman_Yaw_Angle, Kalman_Yaw_Angle_Uncertinity, Yaw_Rate);
  Kalman_Yaw_Angle = Kalman_Output[0];
  Kalman_Yaw_Angle_Uncertinity = Kalman_Output[1];
  
  // Fast slew rate limiter for roll
  float roll_change = Kalman_Roll_Angle - last_roll;
  if (abs(roll_change) > max_change_per_cycle) {
    Kalman_Roll_Angle = last_roll + (roll_change > 0 ? max_change_per_cycle : -max_change_per_cycle);
  }
  last_roll = Kalman_Roll_Angle;
  
  // Fast slew rate limiter for pitch
  float pitch_change = Kalman_Pitch_Angle - last_pitch;
  if (abs(pitch_change) > max_change_per_cycle) {
    Kalman_Pitch_Angle = last_pitch + (pitch_change > 0 ? max_change_per_cycle : -max_change_per_cycle);
  }
  last_pitch = Kalman_Pitch_Angle;
}

void setupMPU() {
  // Initialize the loop timer
  LoopTimer = micros();
  lastMovementTime = LoopTimer;
  
  // Increase I2C speed for faster communication
  Wire.setClock(400000); // Set I2C to 400 kHz
  
  // Set gyro range to ±500°/s
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  
  // Set accelerometer range to ±8g
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  
  // Initialize filter with first accelerometer reading
  readMPUData();
  Kalman_Roll_Angle = Acc_Roll_Angle;
  Kalman_Pitch_Angle = Acc_Pitch_Angle;
  Kalman_Yaw_Angle = 0;
  last_roll = Kalman_Roll_Angle;
  last_pitch = Kalman_Pitch_Angle;
  
  // Get initial temperature
  int16_t temp_raw = mpu.getTemperature();
  baseline_temp = temp_raw / 340.0f + 36.53f;
  current_temp = baseline_temp;
  
  Serial.println("MPU6050 setup complete");
}

// Optimized moving average filter
float applyMovingAverageFilter(float newValue) {
  // Subtract oldest value from sum
  gyro_filter_sum -= GyroZ_history[gyro_history_index];
  // Add new value to sum
  gyro_filter_sum += newValue;
  // Store new value in history
  GyroZ_history[gyro_history_index] = newValue;
  // Update index
  gyro_history_index = (gyro_history_index + 1) % GYRO_FILTER_SIZE;
  
  // Return average
  return gyro_filter_sum * INV_GYRO_FILTER_SIZE;
}

// Improved bias estimation that works better with both positive and negative biases
float estimateGyroBias(float gyroReading) {
  static float bias = 0;
  static float confidence = 0;
  static unsigned long last_time = 0;
  
  // Only update bias when very still
  if (abs(gyroReading) < Gyro_Noise_Threshold * 0.7f) {
    // Lower learning rate for more stability
    float learning_rate = 0.0005f;
    
    // Increase confidence when consecutive stable readings
    if (confidence < 1.0f) {
      confidence += 0.001f;
    }
    
    // Higher confidence = higher learning rate
    learning_rate *= confidence;
    
    // Weighted moving average with tiny learning rate
    bias = bias * (1.0f - learning_rate) + gyroReading * learning_rate;
  } else {
    // Reset confidence when movement detected
    confidence *= 0.9f;
    
    // Only fully reset if major movement
    if (abs(gyroReading) > 5.0f) {
      confidence = 0;
    }
  }
  
  // Record update time
  last_time = micros();
  
  return bias;
}

void readMPUData() {
  int16_t AccX_Raw, AccY_Raw, AccZ_Raw;
  int16_t GyX, GyY, GyZ;
  int16_t temp_raw;
  
  // Get raw sensor data
  mpu.getMotion6(&AccX_Raw, &AccY_Raw, &AccZ_Raw, &GyX, &GyY, &GyZ);
  
  // Only get temperature if compensation is enabled
  if (enable_temp_comp) {
    temp_raw = mpu.getTemperature();
    current_temp = temp_raw / 340.0f + 36.53f;
  }
  
  // Fast accelerometer processing with pre-calculated constants
  AccX = (float)AccX_Raw * INV_4096;
  AccY = (float)AccY_Raw * INV_4096;
  AccZ = (float)AccZ_Raw * INV_4096;
  
  // Apply calibration offsets
  AccX -= Acc_Error_X;
  AccY -= Acc_Error_Y;
  AccZ -= Acc_Error_Z;
  
  // Faster angle calculation
  float accXZ = AccX * AccX + AccZ * AccZ;
  float accYZ = AccY * AccY + AccZ * AccZ;
  if (accXZ < 0.000001f) accXZ = 0.000001f; // Avoid division by zero
  if (accYZ < 0.000001f) accYZ = 0.000001f;
  
  Acc_Pitch_Angle = atan2(AccY, sqrt(accXZ)) * MY_RAD_TO_DEG;
  Acc_Roll_Angle = -atan2(AccX, sqrt(accYZ)) * MY_RAD_TO_DEG;
  
  // Fast gyroscope processing
  GyroX = (float)GyX * INV_65_5;
  GyroY = (float)GyY * INV_65_5;
  GyroZ = (float)GyZ * INV_65_5;
  
  // Apply calibration offsets
  GyroX -= Gyro_Error_X;
  GyroY -= Gyro_Error_Y;
  GyroZ -= Gyro_Error_Z;
  
  // Only apply temperature compensation if enabled
  if (enable_temp_comp) {
    float temp_correction = (current_temp - baseline_temp) * temp_coefficient;
    GyroX -= temp_correction;
    GyroY -= temp_correction;
    GyroZ -= temp_correction;
  }
  
  // Apply correction factor after offsets and temp compensation
  GyroZ *= yaw_correction_factor;
  
  // Apply moving average filter to GyroZ
  GyroZ = applyMovingAverageFilter(GyroZ);
  
  // Apply dynamic bias estimation
  Gyro_Dynamic_Bias_Z = estimateGyroBias(GyroZ);
  GyroZ -= Gyro_Dynamic_Bias_Z;
  
  // Fast threshold filtering - with improved zero region
  if (abs(GyroX) < Gyro_Noise_Threshold) GyroX = 0;
  if (abs(GyroY) < Gyro_Noise_Threshold) GyroY = 0;
  if (abs(GyroZ) < Gyro_Noise_Threshold) GyroZ = 0;
  
  // Store rotation rates
  Roll_Rate = GyroY;
  Pitch_Rate = GyroX;
  Yaw_Rate = GyroZ;
}

void printMPUValues() {
  Serial.println("-------------------------------------");
  Serial.println("Orientation Values:");
  
  Serial.print("Accel-based Roll: ");
  Serial.print(Acc_Roll_Angle);
  Serial.print("°\tPitch: ");
  Serial.print(Acc_Pitch_Angle);
  Serial.println("°");
  
  Serial.print("Filter Roll: ");
  Serial.print(Kalman_Roll_Angle);
  Serial.print("°\tPitch: ");
  Serial.print(Kalman_Pitch_Angle);
  Serial.print("°\tYaw: ");
  Serial.print(Kalman_Yaw_Angle);
  Serial.println("°");
  
  Serial.print("Rotation Rate - Roll: ");
  Serial.print(Roll_Rate);
  Serial.print("°/s\tPitch: ");
  Serial.print(Pitch_Rate);
  Serial.print("°/s\tYaw: ");
  Serial.print(Yaw_Rate);
  Serial.println("°/s");
  
  Serial.print("Gyro Z-axis bias: ");
  Serial.print(Gyro_Dynamic_Bias_Z);
  
  // Print accelerometer magnitude and current alpha
  Serial.print("\tAccel mag: ");
  Serial.print(sqrt(AccX*AccX + AccY*AccY + AccZ*AccZ), 3);
  Serial.print("g  Alpha: ");
  Serial.print(alpha, 3);
  
  if (isStable) {
    Serial.print("\tStable for: ");
    Serial.print((micros() - lastStableTime) / 1000000.0);
    Serial.print("s");
  }
  
  if (drift_correction_enabled) {
    Serial.print("\tDrift correction ACTIVE");
  }
  
  Serial.println();
  Serial.println("-------------------------------------");
}

void calibrateMPU() {
  Serial.println("Calibrating MPU6050 for DMP...");
  Serial.println("Keep the sensor still and level.");
  delay(3000);
  
  int16_t AccX_Raw, AccY_Raw, AccZ_Raw;
  int16_t GyX, GyY, GyZ;
  int16_t temp_raw;
  float accX_sum = 0, accY_sum = 0, accZ_sum = 0;
  float gyroX_sum = 0, gyroY_sum = 0, gyroZ_sum = 0;
  float temp_sum = 0;
  int validSamples = 0;
  const int totalSamples = 5000;
  
  for (int i = 0; i < totalSamples; i++) {
    mpu.getMotion6(&AccX_Raw, &AccY_Raw, &AccZ_Raw, &GyX, &GyY, &GyZ);
    temp_raw = mpu.getTemperature();
    
    float gyroZ_temp = (float)GyZ * INV_65_5;
    
    if (abs(gyroZ_temp) < 5.0f) {
      accX_sum += (float)AccX_Raw * INV_4096;
      accY_sum += (float)AccY_Raw * INV_4096;
      accZ_sum += (float)AccZ_Raw * INV_4096;
      
      gyroX_sum += (float)GyX * INV_65_5;
      gyroY_sum += (float)GyY * INV_65_5;
      gyroZ_sum += gyroZ_temp;
      
      temp_sum += temp_raw / 340.0f + 36.53f;
      
      validSamples++;
    }
    
    if (i % 500 == 0) Serial.print(".");
    delay(1);
  }
  
  if (validSamples > 0) {
    float inv_validSamples = 1.0f / validSamples;
    
    Acc_Error_X = accX_sum * inv_validSamples;
    Acc_Error_Y = accY_sum * inv_validSamples;
    Acc_Error_Z = (accZ_sum * inv_validSamples) - 1.0f;
    
    Gyro_Error_X = gyroX_sum * inv_validSamples;
    Gyro_Error_Y = gyroY_sum * inv_validSamples;
    Gyro_Error_Z = gyroZ_sum * inv_validSamples;
    
    baseline_temp = temp_sum * inv_validSamples;
  } else {
    Serial.println("Error: No valid samples for calibration!");
    return;
  }
  
  Gyro_Dynamic_Bias_Z = Gyro_Error_Z;
  
  // Reset moving average filter
  gyro_filter_sum = 0;
  for (int i = 0; i < GYRO_FILTER_SIZE; i++) {
    GyroZ_history[i] = 0;
  }
  
  Serial.println("\nCalibration complete!");
  Serial.print("Accel Offsets: X=");
  Serial.print(Acc_Error_X, 4);
  Serial.print(" Y=");
  Serial.print(Acc_Error_Y, 4);
  Serial.print(" Z=");
  Serial.println(Acc_Error_Z, 4);
  
  Serial.print("Gyro Offsets: X=");
  Serial.print(Gyro_Error_X, 4);
  Serial.print(" Y=");
  Serial.print(Gyro_Error_Y, 4);
  Serial.print(" Z=");
  Serial.println(Gyro_Error_Z, 4);
  
  delay(1000);
}

void updateAngles() {
  // Read sensor data
  readMPUData();
  
  // Apply ultra-low-latency filter approach
  updateRollPitchAngles();
  
  // Improved stability detection using combined threshold
  // Now includes the absolute value of the Desired_Yaw_Rate
  bool currentlyStable = (abs(Roll_Rate) + abs(Pitch_Rate) + abs(Yaw_Rate) + abs(Desired_Yaw_Rate) < 0.16f);
  
  if (currentlyStable) {
    stable_reading_count++;
    
    if (stable_reading_count >= REQUIRED_STABLE_READINGS) {
      // We've had enough consecutive stable readings
      if (!isStable) {
        isStable = true;
        lastStableTime = micros();
      }
      
      // Check if we've been stable long enough to start drift correction
      if ((int32_t)(micros() - lastStableTime) > DRIFT_DETECTION_TIME && !drift_correction_enabled) {
        drift_correction_enabled = true;
        drift_detection_start = micros();
      }
      
      // If drift correction is active and no movement in very long time (15 seconds)
      if (drift_correction_enabled && (int32_t)(micros() - lastMovementTime) > 15000000) {
        // Complete reset only after extreme stationary period
        Kalman_Yaw_Angle = 0;
        Kalman_Yaw_Angle_Uncertinity = 2 * 2;
        drift_correction_enabled = false;
      }
    }
  } else {
    // Movement detected - disable drift correction immediately
    stable_reading_count = 0;
    isStable = false;
    drift_correction_enabled = false;
    lastMovementTime = micros();
  }
  
  // Fast boundary check for yaw angle
  if (Kalman_Yaw_Angle > 180.0f) {
    Kalman_Yaw_Angle -= 360.0f;
  } else if (Kalman_Yaw_Angle < -180.0f) {
    Kalman_Yaw_Angle += 360.0f;
  }
}

#endif // READ_YPR_H
