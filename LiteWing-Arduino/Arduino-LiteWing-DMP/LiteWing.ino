#include <Wire.h>
#include "read_YPR.h"
#include "PID.h"
#include "motors.h"
#include "ESP_NOW.h"


unsigned long timer = 0;
unsigned long last_loop_time = 0;
float loop_time = 0.01; // Default 10ms

//Variables to send telmetry data via ESP-NOW
uint8_t receiverMacAddress[] = {0x08, 0xB6, 0x1F, 0x81, 0x20, 0xA4}; //address of rcvr
esp_now_peer_info_t peerInfo;
telemetry_data telemetryData;

// Create a struct_message called joystickData
joystick_data joystickData;


void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Init ESP-NOW
  initESPNow();

  // Register callback function for receiving data
  esp_now_register_recv_cb(OnDataRecv);

  // Initialize I2C communication with specific pins
  Wire.begin(11, 10);     // SDA on GPIO11, SCL on GPIO10
  Wire.setClock(400000);  // Fast I2C at 400kHz

    // Initialize the Kalman filter
  setupMPU();

  // Optional: Uncomment to calibrate the MPU6050
  calibrateMPU();

  // Initialize motors
  initializeMotors();

  Serial.println("Setup complete. Starting readings...");
  delay(1000);

  last_loop_time = micros();
}

void sendTelemetry() {
  // Format: "TELEMETRY,desired_roll,desired_pitch,roll_kalman,pitch_kalman,roll_rate,pitch_rate,yaw_rate,desired_yaw_rate,roll_out,pitch_out,yaw_out,throttle,m1,m2,m3,m4"
  Serial.print("TELEMETRY,");
  
  // Desired angles
  Serial.print(Desired_Roll_Angle);
  Serial.print(",");
  Serial.print(Desired_Pitch_Angle);
  Serial.print(",");
  
  // Actual filtered angles
  Serial.print(Kalman_Roll_Angle);
  Serial.print(",");
  Serial.print(Kalman_Pitch_Angle);
  Serial.print(",");
  Serial.print(Kalman_Yaw_Angle);
  Serial.print(",");
  
  // Actual rotation rates
  Serial.print(Roll_Rate);
  Serial.print(",");
  Serial.print(Pitch_Rate);
  Serial.print(",");
  Serial.print(Yaw_Rate);
  Serial.print(",");
  
  // Desired yaw rate
  Serial.print(Desired_Roll_Rate);
  Serial.print(",");
  Serial.print(Desired_Pitch_Rate);
  Serial.print(",");
  Serial.print(Desired_Yaw_Rate);
  Serial.print(",");
  
  // PID outputs
  Serial.print(Roll_PID_Output);
  Serial.print(",");
  Serial.print(Pitch_PID_Output);
  Serial.print(",");
  Serial.print(Yaw_PID_Output);
  Serial.print(",");
  
  // Throttle
  Serial.print(throttle);
  Serial.print(",");
  
  // Motor outputs
  Serial.print(motor1_value);
  Serial.print(",");
  Serial.print(motor2_value);
  Serial.print(",");
  Serial.print(motor3_value);
  Serial.print(",");
  Serial.println(motor4_value);
  
  // Now also send the same data via ESP-NOW
  sendTelemetryESPNow(
    Desired_Roll_Angle, Desired_Pitch_Angle,
    Kalman_Roll_Angle, Kalman_Pitch_Angle,Kalman_Yaw_Angle,
    Roll_Rate, Pitch_Rate, Yaw_Rate,Desired_Roll_Rate, Desired_Pitch_Rate,
    Desired_Yaw_Rate,
    Roll_PID_Output, Pitch_PID_Output, Yaw_PID_Output,
    throttle, motor1_value, motor2_value, motor3_value, motor4_value
  );
}



void loop() {
  // Calculate loop time for accurate PID
  unsigned long current_time = micros();
  loop_time = (current_time - last_loop_time) / 1000000.0; // Convert to seconds
  last_loop_time = current_time;

  // Sanity check on loop time
  if (loop_time > 0.1) loop_time = 0.01; // If time jump is unreasonable, use default
  
  // Read and process sensor data
  updateAngles();

  // Calculate PID values
  calculateAnglePID();
  calculateRatePID();
  calculateMotorOutputs();
  writeToMotors();

  // Send telemetry data for visualization
  if (millis() - timer > 50) {
    sendTelemetry();

  // Print Kalman-filtered angles
  /*Serial.print("Kalman Roll: ");
  Serial.print(Kalman_Roll_Angle);
  Serial.print("°\tPitch: ");
  Serial.print(Kalman_Pitch_Angle);
  Serial.println("°");*/

    // Print the received values
    //Serial.print(joystickData.x1);
    //Serial.print(",");
    //Serial.print(joystickData.x2);
    //Serial.print(",");
    //Serial.println(joystickData.y2);

    timer = millis();
  }
}