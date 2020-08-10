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
Servo motor0;
Servo motor1;
Servo motor2;
Servo motor3;
float motor_control_vector[4];

// Interrupt flag for MPU 9250
volatile bool interrupt_flag_MPU9250 = false;

// Timer
uint32_t last_update_microseconds = 0; // used to calculate integration interval
float euler_angles[3];                 // yaw, pitch, roll (3-2-1)

float quaternion_vector[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion

// initialize global junk
volatile unsigned long last_rise_time[NUM_INPUTS];
volatile float input_radio_values[NUM_INPUTS];
bool is_initializing;

/**************************************************************
 * Function: EstimateStateAndControlVec
**************************************************************/
inline void EstimateStateAndControlVec()
{

    float accel_vector_xyz[3], g[3], magnetometer_vector[3]; // variables to hold latest sensor data values

    // integration interval for both filter schemes
    const uint32_t now_microseconds = micros();
    float delta_time_sec = ((now_microseconds - last_update_microseconds) / 1000000.0f); // set integration time by time elapsed since last filter update
    last_update_microseconds = now_microseconds;

    ReadMPU9250(accel_vector_xyz, g, magnetometer_vector);
    UpdateState(accel_vector_xyz, g, magnetometer_vector, quaternion_vector, delta_time_sec, euler_angles);

    CalculateControlVector(euler_angles, g, motor_control_vector, delta_time_sec);
}

/**************************************************************
 * Function: setup
**************************************************************/
void setup()
{
    is_initializing = true;

    // Setup Serial Comms
    Serial.begin(115200);

    Serial.println(F("INITIALIZING..."));

    // Setup motor control
    motor0.attach(A0);
    motor1.attach(A1);
    motor2.attach(A2);
    motor3.attach(A3);

    // Set up the MPU 9250 IMU board
    SetupMPU9250();

    // Set up the radio reciever
    SetupRadioReceiver(input_radio_values);

    pinMode(LED_LEVEL_INDICATOR, OUTPUT);

    EstimateStateAndControlVec();
    Serial.print(F("Initial Pitch = "));
    Serial.println(euler_angles[1]);
    Serial.print(F("Initial Roll = "));
    Serial.println(euler_angles[2]);
    while (abs(euler_angles[1]) > 2.0)
    {
        EstimateStateAndControlVec();
        // Serial.print("Waiting on PITCH to stabilize: ");
        // Serial.println(euler_angles[1]);
    }
    while (abs(euler_angles[2]) > 2.0)
    {
        EstimateStateAndControlVec();
        // Serial.print("Waiting on ROLL to stabilize: ");
        // Serial.println(euler_angles[2]);
    }
    Serial.println(F("--------------------------------------------------"));
    Serial.println(F("*** PITCH and ROLL have stabilized! ***"));
    Serial.print(F("Pitch = "));
    Serial.println(euler_angles[1]);
    Serial.print(F("Roll = "));
    Serial.println(euler_angles[2]);
    Serial.println(F("--------------------------------------------------"));

    is_initializing = false;
}

/**************************************************************
 * Function: loop
**************************************************************/
void loop()
{
    EstimateStateAndControlVec();

    // Serial.print("|");
    // Serial.print(radioRecieverVals[0]);
    // Serial.print("\t");
    // Serial.print(radioRecieverVals[1]);
    // Serial.print("\t");
    // Serial.print(radioRecieverVals[2]);
    // Serial.print("\t");
    // Serial.print(radioRecieverVals[3]);
    // Serial.print("\t");
    // Serial.print(radioRecieverVals[4]);
    // Serial.println("");

    // esc0.write(motor_control_vector[0]);
    // esc1.write(motor_control_vector[1]);
    // esc2.write(motor_control_vector[2]);
    // esc3.write(motor_control_vector[3]);
}