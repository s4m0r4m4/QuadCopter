#include <Arduino.h>


const unsigned long timeout_limit = 100000;
const float deg2rad = PI / 180.0f;

// ###############################################################################################
void calculate_control_vec()
{
  // Derivitive feedback term
  float kD = 0.1114; //0.1431; //0.0385; //0.01;
  // Propoortional feedback term
  float kP = 0.9487;//1.5652;// 0.1118; //0.3;
  // Reference adjustment
  float Nbar = kP; //0.1118; //

  float pitch = 0.0;
  float roll = 0;
  float pitch_rate = 0.0;
  float roll_rate = 0.0;
  float deltaF_pitch;
  float deltaF_roll;
  // float pitch_err = 0.0;
  // float roll_err = 0.0;
  // float pitch_rate_err = 0.0;
  // float roll_rate_err = 0.0;

  // use Euler angles to calculate errors only if within certain range
  if ((abs(euler_angles[1])<90) && (abs(euler_angles[2])<90)){
    pitch = euler_angles[1];
    roll = euler_angles[2];
    pitch_rate = g[2]; // Through trial and error
    roll_rate = g[1]; // Through trial and error
  }

    // Read primary throttle value from left stick
    valLeftThrottle = radioRecieverVals[pinLeftThrottle];

  // // use Euler angles to calculate errors only if within certain range
  if ((abs(euler_angles[1])<90) && (abs(euler_angles[2])<90)){
      // if euler_angles[1] is high, push on A3 and A0
      // if euler_angles[2] is high, push on A1 and A0
    //   pitch_err = radioRecieverVals[pinRightThrottleUpDown]-euler_angles[1];
    //   roll_err = radioRecieverVals[pinRightThrottleLeftRight]-euler_angles[2];
    //   pitch_rate_err = 0-g[2]; // Through trial and error
    //   roll_rate_err = 0-g[1]; // Through trial and error

    // u = Nbar*r - K*x
    deltaF_pitch = (radioRecieverVals[pinRightThrottleUpDown]*Nbar - pitch*kP)*deg2rad + (0.0f*Nbar - pitch_rate*kD)*deg2rad;
    deltaF_roll = (radioRecieverVals[pinRightThrottleLeftRight]*Nbar - roll*kP)*deg2rad + (0.0f*Nbar - roll_rate*kD)*deg2rad;
  }

  // ratio
  if (check_for_active_signal()==true){
    escControlVec[0] = valLeftThrottle + thrust2val(-deltaF_pitch - deltaF_roll, valLeftThrottle); //- pitch_err*kP - roll_err*kP - pitch_rate_err*kD - roll_rate_err*kD;
    escControlVec[1] = valLeftThrottle + thrust2val(deltaF_pitch - deltaF_roll, valLeftThrottle); // pitch_err*kP - roll_err*kP + pitch_rate_err*kD - roll_rate_err*kD;
    escControlVec[2] = valLeftThrottle + thrust2val(deltaF_pitch + deltaF_roll, valLeftThrottle); // pitch_err*kP + roll_err*kP + pitch_rate_err*kD + roll_rate_err*kD;
    escControlVec[3] = valLeftThrottle + thrust2val(-deltaF_pitch + deltaF_roll, valLeftThrottle); //- pitch_err*kP + roll_err*kP - pitch_rate_err*kD + roll_rate_err*kD;
    } else {
    // if no radio signal, power off the motors
    escControlVec[0] = 0.0;
    escControlVec[1] = 0.0;
    escControlVec[2] = 0.0;
    escControlVec[3] = 0.0;
  }

  if (dumbCounter>50){
    Serial.print(radioRecieverVals[pinRightThrottleUpDown]); Serial.print("\t");
    Serial.print(pitch); Serial.print("\t");
    Serial.print(radioRecieverVals[pinRightThrottleLeftRight]); Serial.print("\t");
    Serial.print(roll); Serial.print("\t");
    Serial.print("|\t");
    Serial.print(pitch_rate); Serial.print("\t");
    Serial.print(roll_rate); Serial.print("\t");
    Serial.print("|\t");

  //  Serial.print(pitch_err); Serial.print("\t");
  //  Serial.print(roll_err); Serial.print("\t");
    // Serial.print(pitch_rate_err); Serial.print("\t");
    // Serial.print(roll_rate_err); Serial.print("\t");
    Serial.print(deltaF_pitch); Serial.print("\t");
    Serial.print(deltaF_roll); Serial.print("\t");
    Serial.print(thrust2val(deltaF_pitch, valLeftThrottle)); Serial.print("\t");
    Serial.print(thrust2val(deltaF_roll, valLeftThrottle)); Serial.print("\t");
    Serial.print("|\t");
   serialPrintArray4(escControlVec);
   // serialPrintArray(euler_angles);
   Serial.print("\n");
    dumbCounter = 0;
  } else {
    dumbCounter++;
  }
}



bool check_for_active_signal(){
  // Serial.print("| ControllerCheck = ");
  // Serial.print(prev_times[pinLeftThrottle]); Serial.print("\t");
  // Serial.print("| micros()-prev_times = ");
  // Serial.println(micros()-prev_times[pinLeftThrottle]);

  if (micros()-prev_times[pinLeftThrottle]> timeout_limit) //(micros()-prev_times[pinRightThrottleLeftRight]>limit) || (micros()-prev_times[pinRightThrottleUpDown]>limit) ||
  {
    return false;
  } else {
    return true;
  }
}

float thrust2val(float deltaThrust, float val0){
  // max RPM of 14618 (go from 0 to max if throttle from 40 to 180)
  const float rpm2val = (180.0-40.0)/(14618.0-0.0);
  const float thrust2rpm = (14618.0-0.0)/(3.5-0);
  // float deltaOmega;
  // // float omega0 = max(val0,40)/rpm2val;
  // float omega0 = max(map(val0, 40.0, 180.0, 0.0, 14600.0),1000);
  // //Serial.print(omega0); Serial.print("\t");
  // deltaOmega = deltaThrust/(2*omega0*18.00201E-09);
  // // deltaOmega = deltaThrust/(2*omega0*9.00201E-09);
  //
  // return deltaOmega*rpm2val;
  return deltaThrust*thrust2rpm*rpm2val;
}

// float thrust2val(float deltaThrust){
//   float deltaOmega;
//   if (deltaThrust>0){
//     deltaOmega = sqrt(deltaThrust)*11664.14848; ///0.0258; // omega in revolutions per minute (RPM)
//   } else {
//     deltaOmega = -sqrt(-deltaThrust)*11664.14848; ///0.0258; // omega in revolutions per minute (RPM)
//   }
//   // max RPM of 14618 (go from 0 to max if throttle from 40 to 180)
//   const float rpm2val = (180.0-40.0)/(14618.0-0.0);
//   return deltaOmega*rpm2val;
// }
