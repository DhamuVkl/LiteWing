//Two Step PID control Step 1 - Angle Controller 2- Rate Controller
//Angle - Refers to tilt of drone (attitude), measured in degrees
//Rate - How fast the tile happened (angular velocity), measured in degrees/second

#ifndef PID_H
#define PID_H

#include "read_YPR.h"

extern float loop_time;

// Desired angles and rotation rates
float Desired_Roll_Angle = 0;  // How tilted the drone should be on Roll
float Desired_Pitch_Angle = 0; // How tiled the drone should be on pitch
float Desired_Yaw_Angle = 0;    // No rotation around Z-axis

// PID Variables for Angle (Stabilize) Mode
// Roll Angle PID
float P_Roll_Angle = 5; //Value to be tuned
float I_Roll_Angle = 0.0; //value to be tuned
float D_Roll_Angle = 0.1; //value to be tuned
float Roll_Angle_Error = 0; //Diff. between kalman roll and desired roll
float Prev_Roll_Angle_Error = 0;
float Roll_Angle_I_term = 0; //To prevent I value accumulating 
float Desired_Roll_Rate = 0; //How fast should the roll be corrected (angular velocity)

// Pitch Angle PID
float P_Pitch_Angle = P_Roll_Angle; //same PID tuning values for Roll and Pitch 
float I_Pitch_Angle = I_Roll_Angle; //same PID tuning values for Roll and Pitch 
float D_Pitch_Angle = D_Roll_Angle; //same PID tuning values for Roll and Pitch 
float Pitch_Angle_Error = 0; //Diff. between kalman pitch and desired pitch
float Prev_Pitch_Angle_Error = 0;
float Pitch_Angle_I_term = 0; //To prevent I value accumulating 
float Desired_Pitch_Rate = 0; //How fast should the pitch be corrected (angular velocity)

// Yaw Angle PID
float P_Yaw_Angle = 8; //Value to be tuned
float I_Yaw_Angle = 0.1; //value to be tuned
float D_Yaw_Angle = 0.01; //value to be tuned
float Yaw_Angle_Error = 0; //Diff. between kalman yaw and desired yaw
float Prev_Yaw_Angle_Error = 0;
float Yaw_Angle_I_term = 0; //To prevent I value accumulating 
float Desired_Yaw_Rate = 0; //How fast should the yaw be corrected (angular velocity)

// PID Variables for Rate Mode
// Roll Rate PID
float P_Roll_Rate = 0.8;
float I_Roll_Rate = 0.0;
float D_Roll_Rate = 0.1;
float Roll_Rate_Error = 0, Prev_Roll_Rate_Error = 0;
float Roll_Rate_I_term = 0;
float Roll_PID_Output = 0;

// Pitch Rate PID
float P_Pitch_Rate = P_Roll_Rate;
float I_Pitch_Rate = I_Roll_Rate;
float D_Pitch_Rate = D_Roll_Rate;
float Pitch_Rate_Error = 0, Prev_Pitch_Rate_Error = 0;
float Pitch_Rate_I_term = 0;
float Pitch_PID_Output = 0;

// Yaw Rate PID
float P_Yaw_Rate = 5;
float I_Yaw_Rate = 0.0;
float D_Yaw_Rate = 0.1;
float Yaw_Rate_Error = 0, Prev_Yaw_Rate_Error = 0;
float Yaw_Rate_I_term = 0;
float Yaw_PID_Output = 0;

// Trim values (small adjustments to compensate for physical imbalances), change this if you drone drifts
float Roll_Trim = 0.0;
float Pitch_Trim = 0.0;

void calculateAnglePID() {
  // Roll angle error
  Roll_Angle_Error = Desired_Roll_Angle - Kalman_Roll_Angle - Roll_Trim;

  // P term
  float p_term = P_Roll_Angle * Roll_Angle_Error;

  // I term with anti-windup
  Roll_Angle_I_term += I_Roll_Angle * Roll_Angle_Error * loop_time;
  Roll_Angle_I_term = constrain(Roll_Angle_I_term, -800, 800);

  // D term
  float d_term = D_Roll_Angle * ((Roll_Angle_Error - Prev_Roll_Angle_Error) / loop_time);

  //### Calculate desired roll rate###
  Desired_Roll_Rate = p_term + Roll_Angle_I_term + d_term; 
  Desired_Roll_Rate = constrain(Desired_Roll_Rate, -800, 800);

  // Store current error for next iteration
  Prev_Roll_Angle_Error = Roll_Angle_Error;

  // Same calculations for pitch
  Pitch_Angle_Error = Desired_Pitch_Angle - Kalman_Pitch_Angle - Pitch_Trim;
  p_term = P_Pitch_Angle * Pitch_Angle_Error;
  Pitch_Angle_I_term += I_Pitch_Angle * Pitch_Angle_Error * loop_time;
  Pitch_Angle_I_term = constrain(Pitch_Angle_I_term, -800, 800);
  d_term = D_Pitch_Angle * ((Pitch_Angle_Error - Prev_Pitch_Angle_Error) / loop_time);

  //### Calculate desired Pitch rate###
  Desired_Pitch_Rate = p_term + Pitch_Angle_I_term + d_term;
  Desired_Pitch_Rate = constrain(Desired_Pitch_Rate, -800, 800);

  // Store current error for next iteration
  Prev_Pitch_Angle_Error = Pitch_Angle_Error;

  // Same calculations for yaw
  Yaw_Angle_Error = Desired_Yaw_Angle - Kalman_Yaw_Angle;
  p_term = P_Yaw_Angle * Yaw_Angle_Error;
  Yaw_Angle_I_term += I_Yaw_Angle * Yaw_Angle_Error * loop_time;
  Yaw_Angle_I_term = constrain(Yaw_Angle_I_term, -800, 800);
  d_term = D_Yaw_Angle * ((Yaw_Angle_Error - Prev_Yaw_Angle_Error) / loop_time);

  //### Calculate desired Pitch rate###
  Desired_Yaw_Rate = p_term + Yaw_Angle_I_term + d_term;
  Desired_Yaw_Rate = constrain(Desired_Yaw_Rate, -800, 800);

  // Store current error for next iteration
  Prev_Yaw_Angle_Error = Yaw_Angle_Error;
}

void calculateRatePID() {
  // Roll rate error
  Roll_Rate_Error = Desired_Roll_Rate - Roll_Rate;

  // P term
  float p_term = P_Roll_Rate * Roll_Rate_Error;

  // I term with anti-windup
  Roll_Rate_I_term += I_Roll_Rate * Roll_Rate_Error * loop_time;
  Roll_Rate_I_term = constrain(Roll_Rate_I_term, -800, 800);

  // D term
  float d_term = D_Roll_Rate * ((Roll_Rate_Error - Prev_Roll_Rate_Error) / loop_time);

  // Calculate roll output
  Roll_PID_Output = p_term + Roll_Rate_I_term + d_term;
  Roll_PID_Output = constrain(Roll_PID_Output, -800, 800);

  // Store current error for next iteration
  Prev_Roll_Rate_Error = Roll_Rate_Error;

  // Same calculations for pitch
  Pitch_Rate_Error = Desired_Pitch_Rate - Pitch_Rate;
  p_term = P_Pitch_Rate * Pitch_Rate_Error;
  Pitch_Rate_I_term += I_Pitch_Rate * Pitch_Rate_Error * loop_time;
  Pitch_Rate_I_term = constrain(Pitch_Rate_I_term, -800, 800);
  d_term = D_Pitch_Rate * ((Pitch_Rate_Error - Prev_Pitch_Rate_Error) / loop_time);

  Pitch_PID_Output = p_term + Pitch_Rate_I_term + d_term;
  Pitch_PID_Output = constrain(Pitch_PID_Output, -800, 800);

  Prev_Pitch_Rate_Error = Pitch_Rate_Error;

  // Yaw rate PID
  Yaw_Rate_Error = Desired_Yaw_Rate - Yaw_Rate;
  p_term = P_Yaw_Rate * Yaw_Rate_Error;
  Yaw_Rate_I_term += I_Yaw_Rate * Yaw_Rate_Error * loop_time;
  Yaw_Rate_I_term = constrain(Yaw_Rate_I_term, -800, 800);
  d_term = D_Yaw_Rate * ((Yaw_Rate_Error - Prev_Yaw_Rate_Error) / loop_time);

  Yaw_PID_Output = p_term + Yaw_Rate_I_term + d_term;
  Yaw_PID_Output = constrain(Yaw_PID_Output, -800, 800);

  Prev_Yaw_Rate_Error = Yaw_Rate_Error;
}

#endif // PID_H