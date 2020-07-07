#include <Arduino.h>

#include <Wire.h>
#include <Servo.h>
#include <mpu9250_code.h>
#include <state_estimator.h>
#include <quadcopter_constants.h>
#include <global_junk.h>

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
uint32_t lastUpdate = 0; // used to calculate integration interval

float euler_angles[3]; // yaw, pitch, roll (3-2-1)

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion

// initialize global junk
volatile float valLeftThrottle = 0;
volatile float valRightThrottleUpDown = 0;
volatile float valRightThrottleLeftRight = 0;
float integrated_pitch_err = 0.0f; // integral error for Controller
float integrated_roll_err = 0.0f;  // integral error for Controller

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
    float a[3], g[3], m[3]; // variables to hold latest sensor data values

    // integration interval for both filter schemes
    uint32_t Now = micros();
    float delta_time = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    Serial.println("Loop!");
    delay(500);

    read_mpu9250(a, g, m);
    updateState(a, g, m, q, delta_time, euler_angles);

    // readRecieverData();
    // delay(50);

    // TODO: merge euler and g into single state vector!!!
    // calculateControlVector(euler_angles, g, escControlVec, delta_time);

    // esc0.write(escControlVec[0]);
    // esc1.write(escControlVec[1]);
    // esc2.write(escControlVec[2]);
    // esc3.write(escControlVec[3]);
}
