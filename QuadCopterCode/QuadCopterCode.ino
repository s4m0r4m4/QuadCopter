#include <Arduino.h>

#include <Wire.h>
// #include <TimerOne.h>
#include <Servo.h>

#define LED_STABLE 8

Servo esc0;
Servo esc1;
Servo esc2;
Servo esc3;
float escControlVec[4];

// Interrupt flag for MPU 9250 (??)
volatile bool intFlag_MPU9250 = false;

int accel_range = 16;// ACC_FULL_SCALE_16_G;
int gyro_range = 500; //GYRO_FULL_SCALE_500_DPS;
int mag_bits = 16;
int gyro_dlpf = 41; // 0, 41, 92, 184, 250
int accel_dlpf = 41; // 0, 41, 92, 184, 460

// Timer
//volatile float t_last;
const float update_state_deltaT = 1000000;
// Linear position
float x[3]; // x, y, z
// Linear velocity
float v[3]; // vx, vy, vz
float euler_angles[3]; // yaw, pitch, roll (3-2-1)
float a[3], g[3], m[3]; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

float deltat = 0.0f;        // integration interval for both filter schemes

uint32_t lastUpdate = 0;//, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

volatile float valLeftThrottle = 0;
volatile float valRightThrottleUpDown = 0;
volatile float valRightThrottleLeftRight = 0;

#define pinLeftThrottle 9
#define pinRightThrottleUpDown 10
#define pinRightThrottleLeftRight 11
#define NUM_PINS 12 // Number of potential input pins
volatile unsigned long pwm_val = 0;
volatile unsigned long prev_times[NUM_PINS]; //
volatile float radioRecieverVals[NUM_PINS];
uint8_t latest_interrupted_pin;
int dumbCounter = 0;

// #######################################################################
void setup() {
  // Setup Serial Comms
  Serial.begin(115200);

  // Setup motor control
  esc0.attach(A0); // setup esc0 on pin X
  esc1.attach(A1);
  esc2.attach(A2);
  esc3.attach(A3);

  // Initial State Variables
  for(int i = 0; i<3; i++){
    x[i] = 0.0f;
    v[i] = 0.0f;
    euler_angles[i] = 0.0f;
  }

  // Set up the MPU 9250 IMU board
  setup_mpu9250(accel_range, gyro_range, mag_bits, gyro_dlpf, accel_dlpf);

  // Set up the radio reciever
  //setupRadioReceiver();

  pinMode(LED_STABLE, OUTPUT);

  while(millis()<1000){}

  // set integration time by time elapsed since last filter update
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f);
  lastUpdate = Now;

//  //  MadgwickQuaternionUpdate(a[0]/9.81f, a[1]/9.81f, a[2]/9.81f, g[0]*PI/180.0f, g[1]*PI/180.0f, g[2]*PI/180.0f,  m[1], m[0], m[2]);
//  //  MadgwickQuaternionUpdate(a, g, m);
//  //  MahonyQuaternionUpdate(a[0], a[1], a[2], g[0]*PI/180.0f, g[1]*PI/180.0f, g[2]*PI/180.0f, my, mx, mz);
//    MahonyQuaternionUpdate(a, g, m);
//  }
  // Start the initial time
//  t_last = millis()/1000.0f;
//  delay(1000);
}

// #######################################################################
void loop() {

    read_mpu9250();
    updateState();

// readRecieverData();
// delay(50);

    // noInterrupts(); // don't want to interrupt from radio reciever to screw up the calculated control vector
    // calculate_control_vec();
    // interrupts();

    esc0.write(escControlVec[0]);
    esc1.write(escControlVec[1]);
    esc2.write(escControlVec[2]);
    esc3.write(escControlVec[3]);
}