#include <Arduino.h>
#include <quadcopter_constants.h>
#include <controller.h>
#include <global_junk.h>
#include <serial_printing.h>

/**************************************************************
 * Variables
**************************************************************/
byte dumb_counter = 0;
float integrated_pitch_err_rad = 0.0f; // integral error for Controller
float integrated_roll_err_rad = 0.0f;  // integral error for Controller

/**************************************************************
 * Function: thrustToMotorValLinear
**************************************************************/
float thrustToMotorValLinear(float delta_thrust, float val0)
{
    // max RPM of 14618 (go from 0 to max if throttle from 40 to 180)
    const float rpm2val = (180.0 - 40.0) / (14618.0 - 0.0);
    const float thrust2rpm = (14618.0 - 0.0) / (3.5 - 0);
    // float deltaOmega;
    // // float omega0 = max(val0,40)/rpm2val;
    // float omega0 = max(map(val0, 40.0, 180.0, 0.0, 14600.0),1000);
    // //Serial.print(omega0); Serial.print(F("\t"));
    // deltaOmega = deltaThrust/(2*omega0*18.00201E-09);
    // // deltaOmega = deltaThrust/(2*omega0*9.00201E-09);
    //
    // return deltaOmega*rpm2val;
    return delta_thrust * thrust2rpm * rpm2val;
}

// ---------- Nonlinear transformation from desired force to motor command val ------------
const float nMinimumValue = -20.0;
const float nLinearToQuadtraicTransition = 40.0; // value at which the curve-fit transitions from linear to quadratic
const float nForceAtLinearToQuadratic = 0.10;    // pseudo-force when motors are driven with a value at the switch from linear to quadratic
const float nLinearSlope = (3.5 - nForceAtLinearToQuadratic) / ((180.0 - nLinearToQuadtraicTransition) * (180.0 - nLinearToQuadtraicTransition));

/**************************************************************
 * Function: motorValToThrustNonlinear
**************************************************************/
float motorValToThrustNonlinear(float val0)
{
    if (val0 > nLinearToQuadtraicTransition)
    {
        return nLinearSlope * (val0 - nLinearToQuadtraicTransition) * (val0 - nLinearToQuadtraicTransition) + nForceAtLinearToQuadratic;
    }
    else
    {
        return (val0 - nMinimumValue) * nForceAtLinearToQuadratic / (nLinearToQuadtraicTransition - nMinimumValue);
    }
}

/**************************************************************
 * Function: thrustToMotorValNonlinear
**************************************************************/
float thrustToMotorValNonlinear(float delta_thrust, float val0)
{
    float thrust = max(0, delta_thrust + motorValToThrustNonlinear(val0));
    float motor_val_out;

    if (thrust > nForceAtLinearToQuadratic)
    {
        // quadratic portion of curve
        motor_val_out = sqrt((thrust - nForceAtLinearToQuadratic) / nLinearSlope) + nLinearToQuadtraicTransition;
    }
    else
    {
        // linear regime of fit curve
        motor_val_out = thrust / (nForceAtLinearToQuadratic / (nLinearToQuadtraicTransition - nMinimumValue)) + nMinimumValue;
    }

    return constrain(motor_val_out, 0.0, 180.0);
}

/**************************************************************
 * Function: calculateControlVector
**************************************************************/
void CalculateControlVector(float *euler_angles, float *angular_rates_deg, float *motor_control_vector, float delta_time)
{
    // ----------- LQR state gains -----------
    // // Derivitive feedback term
    // const float kD = 0.243/4.0; //3.43; //0.1114; //0.1431; //0.0385; //0.01;
    // // Propoortional feedback term
    // const float kP = 4.243/4.0;//6.83;// 0.9487;//1.5652;// 0.1118; //0.3;
    // // Reference adjustment
    // const float Nbar = kP; //0.1118;
    // ---------------------------------------

    // ----------- PD/PID state gains -----------
    // const float kD = 0.04769*2*radioRecieverVals[INDEX_LEFT_KNOB]/100.0; // Derivitive feedback term
    // const float kP = 0.6212*2*radioRecieverVals[INDEX_LEFT_KNOB]/100.0;// Propoortional feedback term
    // const float kP = 0.07;  //*radioRecieverVals[INDEX_LEFT_KNOB]/100.0;// Propoortional feedback term
    // const float kI = 0.55;  // Integral feedback term
    // const float kD = 0.055; // *radioRecieverVals[INDEX_LEFT_KNOB]/100.0; // Derivitive feedback term

    // From Xcos simulation
    const float kP = 0.05;
    const float kI = 0.015;
    const float kD = 0.02;

    // Reference adjustment
    // const float Nbar = kP; //0.1118;
    // ---------------------------------------

    float delta_force_pitch;
    float delta_force_roll;
    float pitch_err_rad = 0.0;
    float roll_err_rad = 0.0;

    // // use Euler angles to calculate errors only if within certain range
    if ((abs(euler_angles[1]) < 85) && (abs(euler_angles[2]) < 85))
    {

        // Only send commands to the motor if recieving a radio signal
        // This provides a nice remote kill switch
        if (checkForActiveSignal())
        {

            if (is_initializing)
            {
                // if we're initializing, just send the raw left throttle
                motor_control_vector[0] = thrustToMotorValNonlinear(0, input_radio_values[INDEX_LEFT_STICK]);
                motor_control_vector[2] = thrustToMotorValNonlinear(0, input_radio_values[INDEX_LEFT_STICK]);
                motor_control_vector[1] = thrustToMotorValNonlinear(0, input_radio_values[INDEX_LEFT_STICK]);
                motor_control_vector[3] = thrustToMotorValNonlinear(0, input_radio_values[INDEX_LEFT_STICK]);
            }
            else
            {

                // TODO: move each control approach into its own function
                // --- LQR Controller design ---
                // u = Nbar*r - K*x
                // deltaF_pitch = (radioRecieverVals[INDEX_RIGHT_STICK_UPDOWN]*Nbar - pitch*kP)*DEG2RAD + (0.0f*Nbar - pitch_rate*kD)*DEG2RAD;
                // deltaF_roll = (radioRecieverVals[INDEX_RIGHT_STICK_LEFTRIGHT]*Nbar - roll*kP)*DEG2RAD + (0.0f*Nbar - roll_rate*kD)*DEG2RAD;
                // ------------------------------

                // --- PID Controller design w/ integrator limit ---
                // scale_val = min(1.0, (input_left_throttle / 100.0) * (input_left_throttle / 100.0));
                pitch_err_rad = (-input_radio_values[INDEX_RIGHT_STICK_UPDOWN] - euler_angles[2]) * DEG2RAD;
                roll_err_rad = (input_radio_values[INDEX_RIGHT_STICK_LEFTRIGHT] - euler_angles[1]) * DEG2RAD;

                // Only accrue integrator error once we're flying (so it doesn't ratchet up on the ground)
                if (input_radio_values[INDEX_LEFT_STICK] > 70)
                {
                    integrated_pitch_err_rad = constrain(integrated_pitch_err_rad * 0.999 + (pitch_err_rad * delta_time), -0.25, 0.25);
                    integrated_roll_err_rad = constrain(integrated_roll_err_rad * 0.999 + (roll_err_rad * delta_time), -0.25, 0.25);
                }

                delta_force_pitch = (pitch_err_rad * kP) + ((0.0f - angular_rates_deg[2]) * DEG2RAD * kD) + (integrated_pitch_err_rad * kI);
                delta_force_roll = (roll_err_rad * kP) + ((0.0f - angular_rates_deg[1]) * DEG2RAD * kD) + (integrated_roll_err_rad * kI);

                // otherwise include pitch and roll force requirements
                motor_control_vector[0] = thrustToMotorValNonlinear(-delta_force_pitch - delta_force_roll, input_radio_values[INDEX_LEFT_STICK]);
                motor_control_vector[2] = thrustToMotorValNonlinear(delta_force_pitch + delta_force_roll, input_radio_values[INDEX_LEFT_STICK]);
                motor_control_vector[1] = thrustToMotorValNonlinear(-delta_force_pitch + delta_force_roll, input_radio_values[INDEX_LEFT_STICK]);
                motor_control_vector[3] = thrustToMotorValNonlinear(delta_force_pitch - delta_force_roll, input_radio_values[INDEX_LEFT_STICK]);
            }
        }
        else
        {
            if (DEBUG_MODE)
            {
                Serial.println(F("-------------- DEAD ------------------"));
            }

            // if no radio signal, power off the motors
            motor_control_vector[0] = 0.0;
            motor_control_vector[1] = 0.0;
            motor_control_vector[2] = 0.0;
            motor_control_vector[3] = 0.0;
        }
    }

    if (dumb_counter > 15 && DEBUG_MODE)
    {

        // char sLine[100];

        // sprintf(sLine, "%05.3f %05.3f %05.3f | %04.1f | %05.3f %05.3f | %05.3f %05.3f | %05.3f %05.3f %05.3f %05.3f\n",
        //         euler_angles[0],
        //         euler_angles[1],
        //         euler_angles[2],
        //         input_radio_values[INDEX_LEFT_STICK],
        //         pitch_err_rad,
        //         integrated_pitch_err_rad,
        //         roll_err_rad,
        //         integrated_roll_err_rad,
        //         motor_control_vector[0],
        //         motor_control_vector[1],
        //         motor_control_vector[2],
        //         motor_control_vector[3]);
        // Serial.print(sLine);

        dumb_counter = 0;
    }
    else
    {
        dumb_counter++;
    }
}

/**************************************************************
 * Function: checkForActiveSignal
**************************************************************/
bool checkForActiveSignal()
{
    const unsigned long TIMEOUT_MICROSECONDS = 100000;
    return (micros() - last_rise_time[INDEX_LEFT_STICK]) < TIMEOUT_MICROSECONDS;
}