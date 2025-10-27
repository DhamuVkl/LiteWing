#ifndef ESP_NOW_H
#define ESP_NOW_H

#include <WiFi.h>
#include <esp_now.h>

// Data structure to receive joystick data
typedef struct joystick_data {
  int x1;
  int x2;
  int y2;
  float rollTrim;
  float pitchTrim;
  float angleP;
  float angleI;
  float angleD;
  float rollPitchP;
  float rollPitchI;
  float rollPitchD;
  float yawP;
  float yawI;
  float yawD;
} joystick_data;

// Data structure for sending telemetry
typedef struct telemetry_data {
  float desired_roll;
  float desired_pitch;
  float roll_kalman;
  float pitch_kalman;
  float yaw_kalman;
  float roll_rate;
  float pitch_rate;
  float yaw_rate;
  float desired_roll_rate;
  float desired_pitch_rate;
  float desired_yaw_rate;
  float roll_out;
  float pitch_out;
  float yaw_out;
  int throttle;
  int m1;
  int m2;
  int m3;
  int m4;
} telemetry_data;

// Global variables
extern joystick_data joystickData;


// ESP-NOW peer info
extern uint8_t receiverMacAddress[];
extern esp_now_peer_info_t peerInfo;
extern telemetry_data telemetryData;

// Forward declaration of callback functions
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

// Function to initialize ESP-NOW
inline void initESPNow() {
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback functions
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

// Callback when data is received
inline void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&joystickData, incomingData, sizeof(joystickData));
  throttle = joystickData.x1;
  Desired_Roll_Angle = joystickData.y2;
  Desired_Pitch_Angle = joystickData.x2;
  Roll_Trim = joystickData.rollTrim;
  Pitch_Trim = joystickData.pitchTrim;

  P_Roll_Angle = P_Pitch_Angle = joystickData.angleP;
  I_Roll_Angle = I_Pitch_Angle = joystickData.angleI;
  D_Roll_Angle = D_Pitch_Angle = joystickData.angleD;

  P_Roll_Rate = P_Pitch_Rate = joystickData.rollPitchP;
  I_Roll_Rate = I_Pitch_Rate = joystickData.rollPitchI;
  D_Roll_Rate = D_Pitch_Rate = joystickData.rollPitchD;

  P_Yaw_Rate = joystickData.yawP;
  I_Yaw_Rate = joystickData.yawI;
  D_Yaw_Rate = joystickData.yawD;


  // Print received values to Serial
  /*Serial.print("throttle: "); Serial.println(throttle);
  Serial.print("Desired_Roll_Angle: "); Serial.println(Desired_Roll_Angle);
  Serial.print("Desired_Pitch_Angle: "); Serial.println(Desired_Pitch_Angle);
  Serial.print("Roll_Trim: "); Serial.println(Roll_Trim);
  Serial.print("Pitch_Trim: "); Serial.println(Pitch_Trim);

  Serial.print("P_Roll_Angle: "); Serial.println(P_Roll_Angle);
  Serial.print("I_Roll_Angle: "); Serial.println(I_Roll_Angle);
  Serial.print("D_Roll_Angle: "); Serial.println(D_Roll_Angle);

  Serial.print("P_Roll_Rate: "); Serial.println(P_Roll_Rate);
  Serial.print("P_Roll_Rate: "); Serial.println(I_Roll_Rate);
  Serial.print("P_Roll_Rate: "); Serial.println(D_Roll_Rate);

  Serial.print("P_Yaw_Rate: "); Serial.println(P_Yaw_Rate);
  Serial.print("I_Yaw_Rate: "); Serial.println(I_Yaw_Rate);
  Serial.print("D_Yaw_Rate: "); Serial.println(D_Yaw_Rate);*/


}

// Callback when data is sent
inline void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("Delivery status: ");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Function to send telemetry data
inline bool sendTelemetryESPNow(
  float d_roll, float d_pitch, 
  float k_roll, float k_pitch, float k_yaw,
  float r_rate, float p_rate, float y_rate,
  float d_r_rate,float d_p_rate,float d_y_rate,
  float r_out, float p_out, float y_out,
  int throt, int m1, int m2, int m3, int m4) {
  
  // Fill the telemetry data structure
  telemetryData.desired_roll = d_roll;
  telemetryData.desired_pitch = d_pitch;
  telemetryData.roll_kalman = k_roll;
  telemetryData.pitch_kalman = k_pitch;
  telemetryData.yaw_kalman = k_yaw;
  telemetryData.roll_rate = r_rate;
  telemetryData.pitch_rate = p_rate;
  telemetryData.yaw_rate = y_rate;
  telemetryData.desired_roll_rate = d_r_rate;
  telemetryData.desired_pitch_rate = d_p_rate;
  telemetryData.desired_yaw_rate = d_y_rate;
  telemetryData.roll_out = r_out;
  telemetryData.pitch_out = p_out;
  telemetryData.yaw_out = y_out;
  telemetryData.throttle = throt;
  telemetryData.m1 = m1;
  telemetryData.m2 = m2;
  telemetryData.m3 = m3;
  telemetryData.m4 = m4;
  
  // Send the data
  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &telemetryData, sizeof(telemetryData));
  
  return result == ESP_OK;
}

#endif // ESP_NOW_H