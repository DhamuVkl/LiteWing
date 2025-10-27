#ifndef MOTORS_H
#define MOTORS_H

#include "PID.h"

// PWM Configuration
#define FREQ 20000          // 20 kHz PWM frequency
#define RESOLUTION 8       // 10-bit resolution (0-1023)
#define MIN_DUTY_CYCLE 100  // Minimum speed duty cycle
#define MIN_THROTTLE 1100   // Minimum throttle to arm motors
#define MAX_THROTTLE 1900   // Maximum throttle value

// Define motor PWM pins //m1-brown, m2-pink, m3-blue, m4-yellow
#define MOTOR_1 5  // Front right
#define MOTOR_2 6  // Back left
#define MOTOR_3 3  // Back right
#define MOTOR_4 4  // Front right

// Default throttle (for testing only - this would normally come from radio control)
int throttle = 100;  // Just above arming threshold for testing

// Motor output values
int motor1_value = 0;
int motor2_value = 0;
int motor3_value = 0;
int motor4_value = 0;

void initializeMotors() {
  // Configure PWM channels for motors
  ledcSetup(0, FREQ, RESOLUTION);
  ledcSetup(1, FREQ, RESOLUTION);
  ledcSetup(2, FREQ, RESOLUTION);
  ledcSetup(3, FREQ, RESOLUTION);

  // Attach PWM channels to GPIOs
  ledcAttachPin(MOTOR_1, 0);
  ledcAttachPin(MOTOR_2, 1);
  ledcAttachPin(MOTOR_3, 2);
  ledcAttachPin(MOTOR_4, 3);

  // Activate each motor sequentially to ensure they're working, order is brown, pink, blue and yellow. motor that spins first is brown
  ledcWrite(0, 50);
  delay (100);
  ledcWrite(0, 0);

  ledcWrite(1, 50);
  delay (100);
  ledcWrite(1, 0);

  ledcWrite(2, 50);
  delay (100);
  ledcWrite(2, 0);

  ledcWrite(3, 50);
  delay (100);
  ledcWrite(3, 0);
}

void calculateMotorOutputs() {
  // Limit throttle to maximum
  if (throttle > MAX_THROTTLE) {
    throttle = MAX_THROTTLE;
  }

  // Calculate motor values
  /* Motor layout:
      M0(FL)    M3(FR)
         \      /
          \    /
           ----
          /    \
         /      \
      M1(BL)    M2(BR)
  */

  float m1 = throttle - Roll_PID_Output - Pitch_PID_Output + Yaw_PID_Output;  // Front Right
  float m2 = throttle - Roll_PID_Output + Pitch_PID_Output - Yaw_PID_Output;  // Back Right
  float m3 = throttle + Roll_PID_Output + Pitch_PID_Output + Yaw_PID_Output;  // Back Left
  float m4 = throttle + Roll_PID_Output - Pitch_PID_Output - Yaw_PID_Output;  // Front Left

  // Map from throttle range (1000-2000) to PWM range (0-1023)
  motor1_value = constrain(map(m1, 1000, 2000, 0, 255), 0, 255); //brown
  motor2_value = constrain(map(m2, 1000, 2000, 0, 255), 0, 255); //pink
  motor3_value = constrain(map(m3, 1000, 2000, 0, 255), 0, 255); //blue
  motor4_value = constrain(map(m4, 1000, 2000, 0, 255), 0, 255); //yellow
}

void writeToMotors() {
  // Only write to motors if armed (throttle > minimum)
  if (throttle > MIN_THROTTLE) {
    ledcWrite(0, motor1_value); //brown
    ledcWrite(1, motor2_value); //pink
    ledcWrite(2, motor3_value); //blue
    ledcWrite(3, motor4_value); //yellow
  } else {
    // Motors off if throttle below minimum
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    ledcWrite(3, 0);

    // Reset I terms to prevent buildup
    Roll_Angle_I_term = 0;
    Pitch_Angle_I_term = 0;
    Roll_Rate_I_term = 0;
    Pitch_Rate_I_term = 0;
    Yaw_Rate_I_term = 0;
  }
}

#endif // MOTORS_H