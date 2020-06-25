#include <Arduino.h>

#include <Wire.h>
#include <Servo.h>
#include <mpu9250_code.h>
#include <state_estimator.h>
#include <quadcopter_constants.h>

/**************************************************************
 * Pin outs for radio and LED indicator
**************************************************************/

Servo esc0;
Servo esc1;
Servo esc2;
Servo esc3;
float escControlVec[4];

// Interrupt flag for MPU 9250
volatile bool intFlag_MPU9250 = false;

// Timer
// volatile float t_last;
const float update_state_deltaT = 1000000;
float euler_angles[3];                 // yaw, pitch, roll (3-2-1)
float a[3], g[3], m[3];                // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion
float eInt[3] = {
    0.0f, 0.0f,
    0.0f};                         // vector to hold integral error for estimator Mahony method
float integrated_pitch_err = 0.0f; // integral error for Controller
float integrated_roll_err = 0.0f;  // integral error for Controller

volatile float valLeftThrottle = 0;
volatile float valRightThrottleUpDown = 0;
volatile float valRightThrottleLeftRight = 0;

volatile unsigned long prev_times[NUM_PINS];
volatile float radioRecieverVals[NUM_PINS];
unsigned long RADIO_STICK_DIFF = 370;
int dumbCounter = 0;

const unsigned long timeout_limit = 100000;
const float DEG2RAD = PI / 180.0f;

/**************************************************************
 * Function: setup
**************************************************************/
void setup()
{

    // Setup Serial Comms
    Serial.begin(115200);

    Serial.println("INITIALIZING...");

    // Setup motor control
    esc0.attach(A0); // setup esc0 on pin X
    esc1.attach(A1);
    esc2.attach(A2);
    esc3.attach(A3);

    // Set up the MPU 9250 IMU board
    setup_mpu9250();

    // Set up the radio reciever
    // setupRadioReceiver();

    pinMode(LED_STABLE, OUTPUT);

    // set integration time by time elapsed since last filter update
    // Now = micros();
    // delta_time = ((Now - lastUpdate) / 1000000.0f);
    // lastUpdate = Now;

    // while (millis() < 1000)
    // {
    //     Serial.println("Testing!!!");
    //     read_mpu9250();
    //     updateState();
    // }

    Serial.print("Initial Pitch = ");
    Serial.println(euler_angles[1]);
    Serial.print("Initial Roll = ");
    Serial.println(euler_angles[2]);
    // while(abs(euler_angles[1])>2.0){
    //   read_mpu9250();
    //   updateState();
    //   Serial.print("Waiting on PITCH to stabilize: ");
    //   Serial.println(euler_angles[1]);
    // }
    // while(abs(euler_angles[2])>2.0){
    //   read_mpu9250();
    //   updateState();
    //   Serial.print("Waiting on ROLL to stabilize: ");
    //   Serial.println(euler_angles[2]);
    // }
    Serial.println("--------------------------------------------------");
    Serial.println("*** PITCH and ROLL have stabilized! ***");
    Serial.print("Pitch = ");
    Serial.println(euler_angles[1]);
    Serial.print("Roll = ");
    Serial.println(euler_angles[2]);
    Serial.println("--------------------------------------------------");
}

/**************************************************************
 * Function: loop
**************************************************************/
void loop()
{
    Serial.println("Loop!");
    delay(500);

    read_mpu9250(a, g, m);
    updateState(a, g, m);

    // readRecieverData();
    // delay(50);

    // calculateControlVector();

    // esc0.write(escControlVec[0]);
    // esc1.write(escControlVec[1]);
    // esc2.write(escControlVec[2]);
    // esc3.write(escControlVec[3]);
}
