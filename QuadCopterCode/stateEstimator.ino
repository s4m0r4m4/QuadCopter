#include <Arduino.h>



// ###############################################################################################       
void updateState() //
{
  
  float pitch, yaw, roll;
  
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

//  MadgwickQuaternionUpdate(a[0]/9.81f, a[1]/9.81f, a[2]/9.81f, g[0]*PI/180.0f, g[1]*PI/180.0f, g[2]*PI/180.0f,  m[1], m[0], m[2]);
//  MadgwickQuaternionUpdate(a, g, m);
//  MahonyQuaternionUpdate(a[0], a[1], a[2], g[0]*PI/180.0f, g[1]*PI/180.0f, g[2]*PI/180.0f, my, mx, mz);
  MahonyQuaternionUpdate(a, g, m);

  adjustAccelData(a, q);
  
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
  yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
  pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  pitch *= 180.0f / PI;
  yaw   *= 180.0f / PI; 
  yaw   -= 12.2; // Declination at Burbank, California is 12.2 degrees
////      yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
  roll  *= 180.0f / PI;

  // Set gloabl variables
  euler_angles[0] = yaw;
  euler_angles[1] = roll;
  euler_angles[2] = pitch;

  if ((roll<5)&&(roll>-5)&&(pitch<5)&&(pitch>-5)){
    digitalWrite(LED_STABLE, LOW);
  } else {
    digitalWrite(LED_STABLE, HIGH);
  }

  float v_avg = 0;
  float cutoff = 0.1;

//if (Now>5200000){
  for (int j = 0; j<3; j++){
      //v_avg = 1.0/2.0*(v[j] + (v[j] + a[j]*deltat));
      if (abs(a[j])>cutoff){
         v[j] = v[j] + a[j]*deltat;
      }
      if (abs(v[j])>cutoff){
        x[j] = x[j] + v[j]*deltat;
      //x[j] += v_avg*deltat;
      }
  }
//} else {
////   Serial.println("WAITING for ACCEL to CALM down");
//}
  
//  Serial.print(Now); Serial.print("\t");
//  serialPrintArray(a);
//  serialPrintArray(v);
//  serialPrintArrayLn(x);
//v[0] = v[0] + 1.0f;  
//  serialPrintArrayLn(x);
  

//      serialPrintArray(q);
//      serialPrintArray(euler_angles);

}

