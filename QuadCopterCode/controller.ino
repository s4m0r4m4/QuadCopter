#include <Arduino.h>

// ###############################################################################################
void calculate_control_vec()
{
  // ----------- LQR state gains -----------
  // // Derivitive feedback term
  // const float kD = 0.243/4.0; //3.43; //0.1114; //0.1431; //0.0385; //0.01;
  // // Propoortional feedback term
  // const float kP = 4.243/4.0;//6.83;// 0.9487;//1.5652;// 0.1118; //0.3;
  // // Reference adjustment
  // const float Nbar = kP; //0.1118;
  // ---------------------------------------

  // ----------- PDF state gains -----------
  // Derivitive feedback term
  const float kD = 0.1045*radioRecieverVals[pinLeftKnob]/100.0;
  // Propoortional feedback term
  const float kP = 2.071*radioRecieverVals[pinLeftKnob]/100.0;
  // Reference adjustment
  const float Nbar = kP; //0.1118;
  // ---------------------------------------

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

    // --- LQR Controller design ---
    // u = Nbar*r - K*x
    // deltaF_pitch = (radioRecieverVals[pinRightThrottleUpDown]*Nbar - pitch*kP)*deg2rad + (0.0f*Nbar - pitch_rate*kD)*deg2rad;
    // deltaF_roll = (radioRecieverVals[pinRightThrottleLeftRight]*Nbar - roll*kP)*deg2rad + (0.0f*Nbar - roll_rate*kD)*deg2rad;
    // ------------------------------

    // --- PDF Controller design ---
    deltaF_pitch = (-radioRecieverVals[pinRightThrottleUpDown] - pitch)*deg2rad*kP + (0.0f - pitch_rate)*deg2rad*kD;
    deltaF_roll = (radioRecieverVals[pinRightThrottleLeftRight] - roll)*deg2rad*kP + (0.0f - roll_rate)*deg2rad*kD;
    // ------------------------------
  }

  // ratio
  if (check_for_active_signal()==true){
    // valLeftThrottle +
    escControlVec[0] = thrust2valNonLinear(-deltaF_pitch - deltaF_roll, valLeftThrottle); //- pitch_err*kP - roll_err*kP - pitch_rate_err*kD - roll_rate_err*kD;
    escControlVec[1] = thrust2valNonLinear(deltaF_pitch - deltaF_roll, valLeftThrottle); // pitch_err*kP - roll_err*kP + pitch_rate_err*kD - roll_rate_err*kD;
    escControlVec[2] = thrust2valNonLinear(deltaF_pitch + deltaF_roll, valLeftThrottle); // pitch_err*kP + roll_err*kP + pitch_rate_err*kD + roll_rate_err*kD;
    escControlVec[3] = thrust2valNonLinear(-deltaF_pitch + deltaF_roll, valLeftThrottle); //- pitch_err*kP + roll_err*kP - pitch_rate_err*kD + roll_rate_err*kD;
    } else {
    // if no radio signal, power off the motors
    escControlVec[0] = 0.0;
    escControlVec[1] = 0.0;
    escControlVec[2] = 0.0;
    escControlVec[3] = 0.0;
  }

  if (dumbCounter>15){
    Serial.print(valLeftThrottle); Serial.print("|\t");
    Serial.print(radioRecieverVals[pinLeftKnob]); Serial.print("\t");
    Serial.print(radioRecieverVals[pinRightKnob]); Serial.print("|\t");
    Serial.print(radioRecieverVals[pinRightThrottleUpDown]); Serial.print("\t");
    Serial.print(radioRecieverVals[pinRightThrottleLeftRight]); Serial.print("|\t");
//
//   //  Serial.print(pitch*kP); Serial.print("\t");
//   //  Serial.print(roll*kP); Serial.print("\t");
//   //  Serial.print("|\t");
//   //  Serial.print(pitch_rate*kD); Serial.print("\t");
//   //  Serial.print(roll_rate*kD); Serial.print("\t");
//   //  Serial.print("|\t");
// //
// //  //  Serial.print(pitch_err); Serial.print("\t");
// //  //  Serial.print(roll_err); Serial.print("\t");
// //    // Serial.print(pitch_rate_err); Serial.print("\t");
// //    // Serial.print(roll_rate_err); Serial.print("\t");
//   //  Serial.print(deltaF_pitch); Serial.print("\t");
//   //  Serial.print(deltaF_roll); Serial.print("\t");
    Serial.print(thrust2valNonLinear(deltaF_pitch, valLeftThrottle)-valLeftThrottle); Serial.print("\t");
    Serial.print(thrust2valNonLinear(deltaF_roll, valLeftThrottle)-valLeftThrottle); Serial.print("\t");

    Serial.print("|\t");
   serialPrintArray4(escControlVec);
//    // serialPrintArray(euler_angles);
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

float thrust2valLinear(float deltaThrust, float val0){
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


// ---------- Nonlinear transformation from desired force to motor command val ------------
const float valMin = -30.0;
const float linToQuad = 40.0;
const float forceAtLinToQuad = 0.15; // pseudo-force when motors are driven with a value at the switch from linear to quadratic
const float maxThrustOverVal = (3.5-forceAtLinToQuad)/((180.0-linToQuad)*(180.0-linToQuad));  // (180-40.0)^2= 19600

float thrust2valNonLinear(float deltaThrust, float val0){
  float thrust = max(0, deltaThrust + val2thrustNonLinear(val0));
  if (thrust>forceAtLinToQuad){
    float valOut = sqrt((thrust-forceAtLinToQuad) / maxThrustOverVal) + linToQuad;
    return constrain(valOut, 0.0, 180.0);
  } else {
    return thrust/(forceAtLinToQuad/(linToQuad-valMin))+valMin;
  }
}

float val2thrustNonLinear(float val0){
  if (val0>linToQuad){
    return maxThrustOverVal*(val0-linToQuad)*(val0-linToQuad)+forceAtLinToQuad;
  } else {
    return (val0-valMin)*forceAtLinToQuad/(linToQuad-valMin);
    // return val0*forceAtLinToQuad/40;
  }
}
