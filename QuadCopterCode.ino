//#include "mpu9250_code"
#include <Wire.h>
#include <TimerOne.h>

// Interrupt flag for MPU 9250 (??)
volatile bool intFlag_MPU9250 = false;

int accel_range = 16;// ACC_FULL_SCALE_16_G;
int gyro_range = 2000; //GYRO_FULL_SCALE_500_DPS; 
int mag_bits = 16; // FIXME

// Timer
volatile float t_last;
const float update_state_deltaT = 1000000;
// 6 DOF State
volatile float x[6];

// #######################################################################
void setup() {
  //Initial state = 0
//  for(int i = 0; i<sizeof(x); i++){
//    x[i] = 0.0f;
//  }

  // Set up the MPU 9250 IMU board
  setup_mpu9250(accel_range, gyro_range, mag_bits);
  
//  pinMode(13, OUTPUT);
  Timer1.initialize(update_state_deltaT);         // initialize timer1, and set a 1/2 second period
//  Timer1.attachInterrupt(callback_UpdateState);  // attaches callback() as a timer overflow interrupt
  
  // Start the initial time
  t_last = millis()/1000.0f;
}

// #######################################################################
void loop() {

  read_mpu9250();  //(a, g, m); //&intFlag_MPU9250,

}

// #######################################################################
void callback_UpdateState()
{ 
//  // Get the current time
//  float t = millis()/1000.0f; // this isn't quite right, its the time since the function started, but we might be ok
////  a[0] = a[0]*10;
//  x[0] = x[0] + (0.5f * a[0] * (t - t_last)*(t - t_last));
//
//  t_last = t;  
}

