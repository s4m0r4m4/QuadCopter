#include <Arduino.h>

// euler_angles[3]; // yaw, pitch, roll (3-2-1)

// ###############################################################################################
void calculate_control_vec()
{
  // Derivitive feedback term
  float kD = 0.01;
  float kP = 0.5;

//  serialPrintArray(euler_angles);

  float pitch_err = 0.0;
  float roll_err = 0.0;
  float pitch_rate_err = 0.0;
  float roll_rate_err = 0.0;

  // use Euler angles to calculate errors only if within certain range
  if ((abs(euler_angles[1])<90) || (abs(euler_angles[2])<90)){    
    pitch_err = valRightThrottleUpDown-euler_angles[1];
    roll_err = valRightThrottleLeftRight-euler_angles[2];
    pitch_rate_err = 0-g[2]; // Through trial and error
    roll_rate_err = 0-g[1]; // Through trial and error
  }

  // if euler_angles[1] is high, push on A3 and A0
  // if euler_angles[2] is high, push on A1 and A0
  if (valLeftThrottle>-10){
    escControlVec[0] = valLeftThrottle - pitch_err*kP - roll_err*kP - pitch_rate_err*kD - roll_rate_err*kD;
    escControlVec[1] = valLeftThrottle + pitch_err*kP - roll_err*kP + pitch_rate_err*kD - roll_rate_err*kD;
    escControlVec[2] = valLeftThrottle + pitch_err*kP + roll_err*kP + pitch_rate_err*kD + roll_rate_err*kD;
    escControlVec[3] = valLeftThrottle - pitch_err*kP + roll_err*kP - pitch_rate_err*kD + roll_rate_err*kD;
  } else {
    // if the radio is off, it will read something like -240, so then we just want to power off the motors
    escControlVec[0] = 0.0;
    escControlVec[1] = 0.0;
    escControlVec[2] = 0.0;
    escControlVec[3] = 0.0;
  }

  Serial.print(pitch_err); Serial.print("\t");
  Serial.print(roll_err); Serial.print("\t");
  Serial.print(pitch_rate_err); Serial.print("\t");
  Serial.print(roll_rate_err); Serial.print("\t");
  serialPrintArray4(escControlVec);
//  serialPrintArray(g);
  Serial.print("\n");
}
