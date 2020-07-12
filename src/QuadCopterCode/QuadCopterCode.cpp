#include <Arduino.h>
#include <controller.h>
#include <Wire.h>
#include <Servo.h>
#include <mpu9250_code.h>
#include <state_estimator.h>
#include <quadcopter_constants.h>
#include <global_junk.h>
#include <receive_radio_signal.h>
#include <serial_printing.h>

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
volatile unsigned long prev_times[NUM_PINS];
volatile float radioRecieverVals[NUM_PINS];

/**************************************************************
 * Function: runStateEstimation
**************************************************************/
inline void runStateEstimation()
{

    float a[3], g[3], m[3]; // variables to hold latest sensor data values

    // integration interval for both filter schemes
    uint32_t Now = micros();
    float delta_time = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    read_mpu9250(a, g, m);
    updateState(a, g, m, q, delta_time, euler_angles);
}

/**************************************************************
 * Function: setup
**************************************************************/
void setup()
{

    // Setup Serial Comms
    Serial.begin(115200);

    Serial.println(F("INITIALIZING..."));

    // Setup motor control
    esc0.attach(A0);
    esc1.attach(A1);
    esc2.attach(A2);
    esc3.attach(A3);

    // Set up the MPU 9250 IMU board
    setup_mpu9250();

    // Set up the radio reciever
    setupRadioReceiver(radioRecieverVals);

    pinMode(LED_STABLE, OUTPUT);

    Serial.print(F("Initial Pitch = "));
    Serial.println(euler_angles[1]);
    Serial.print(F("Initial Roll = "));
    Serial.println(euler_angles[2]);
    while (abs(euler_angles[1]) > 2.0)
    {
        runStateEstimation();
        Serial.print("Waiting on PITCH to stabilize: ");
        Serial.println(euler_angles[1]);
    }
    while (abs(euler_angles[2]) > 2.0)
    {
        runStateEstimation();
        Serial.print("Waiting on ROLL to stabilize: ");
        Serial.println(euler_angles[2]);
    }
    Serial.println(F("--------------------------------------------------"));
    Serial.println(F("*** PITCH and ROLL have stabilized! ***"));
    Serial.print(F("Pitch = "));
    Serial.println(euler_angles[1]);
    Serial.print(F("Roll = "));
    Serial.println(euler_angles[2]);
    Serial.println(F("--------------------------------------------------"));
}

/**************************************************************
 * Function: loop
**************************************************************/
void loop()
{
    runStateEstimation();

    Serial.print("|");
    Serial.print(radioRecieverVals[0]);
    Serial.print("\t");
    Serial.print(radioRecieverVals[1]);
    Serial.print("\t");
    Serial.print(radioRecieverVals[2]);
    Serial.print("\t");
    Serial.print(radioRecieverVals[3]);
    Serial.print("\t");
    Serial.print(radioRecieverVals[4]);
    Serial.print("\t");
    Serial.print(radioRecieverVals[5]);
    Serial.print("\t");
    Serial.print(radioRecieverVals[6]);
    Serial.print("\t");
    Serial.print(radioRecieverVals[7]);
    Serial.print("\t");
    Serial.print(radioRecieverVals[8]);
    Serial.print("\t");
    Serial.print(radioRecieverVals[9]);
    Serial.print("\t");
    Serial.print(radioRecieverVals[10]);
    Serial.print("\t");
    Serial.println(radioRecieverVals[11]);

    delay(50);

    // TODO: merge euler and g into single state vector!!!
    //calculateControlVector(euler_angles, g, escControlVec, delta_time);

    // esc0.write(escControlVec[0]);
    // esc1.write(escControlVec[1]);
    // esc2.write(escControlVec[2]);
    // esc3.write(escControlVec[3]);
}