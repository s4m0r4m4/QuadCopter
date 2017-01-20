// euler_angles[3]; // yaw, pitch, roll (3-2-1)

// ###############################################################################################       
void calculate_control_vec() //
{
  // Derivitive feedback term
  float kD = 2.0;
  
//  serialPrintArray(euler_angles);
  
  // use Euler angles to calculate errors
  float pitch_err = 0;//-euler_angles[1];
  float roll_err = 0;//-euler_angles[2];
 
  // if e[1] is high, push on A3 and A0  
  // if e[2] is high, push on A1 and A0
  escControlVec[0] = valLeftThrottle;// - pitch_err*kD - roll_err*kD;
  escControlVec[1] = valLeftThrottle;// + pitch_err*kD - roll_err*kD;
  escControlVec[2] = valLeftThrottle;// + pitch_err*kD + roll_err*kD;
  escControlVec[3] = valLeftThrottle;// - pitch_err*kD + roll_err*kD;

//  serialPrintArray4(escControlVec);
  Serial.print("\n");
}

