#include <Arduino.h>
#include <quadcopter_constants.h>
#include <controller.h>
#include <global_junk.h>
#include <serial_printing.h>

int dumb_counter = 0;
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
const float valMin = -20.0;
const float linToQuad = 40.0;        // value at which the curve-fit transitions from linear to quadratic
const float forceAtLinToQuad = 0.10; // pseudo-force when motors are driven with a value at the switch from linear to quadratic
const float maxThrustOverVal = (3.5 - forceAtLinToQuad) / ((180.0 - linToQuad) * (180.0 - linToQuad));

/**************************************************************
 * Function: motorValToThrustNonlinear
**************************************************************/
float motorValToThrustNonlinear(float val0)
{
    if (val0 > linToQuad)
    {
        return maxThrustOverVal * (val0 - linToQuad) * (val0 - linToQuad) + forceAtLinToQuad;
    }
    else
    {
        return (val0 - valMin) * forceAtLinToQuad / (linToQuad - valMin);
        // return val0*forceAtLinToQuad/40;
    }
}

/**************************************************************
 * Function: thrustToMotorValNonlinear
**************************************************************/
float thrustToMotorValNonlinear(float delta_thrust, float val0)
{
    float thrust = max(0, delta_thrust + motorValToThrustNonlinear(val0));
    float motor_val_out;

    if (thrust > forceAtLinToQuad)
    {
        // quadratic portion of curve
        motor_val_out = sqrt((thrust - forceAtLinToQuad) / maxThrustOverVal) + linToQuad;
    }
    else
    {
        // linear regime of fit curve
        motor_val_out = thrust / (forceAtLinToQuad / (linToQuad - valMin)) + valMin;
    }

    return constrain(motor_val_out, 0.0, 180.0);
}

/**************************************************************
 * Function: calculateControlVector
**************************************************************/
void CalculateControlVector(float *euler_angles, float *g, float *motor_control_vector, float delta_time)
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
    const float kP = 0.07;  //*radioRecieverVals[INDEX_LEFT_KNOB]/100.0;// Propoortional feedback term
    const float kI = 0.55;  // Integral feedback term
    const float kD = 0.055; // *radioRecieverVals[INDEX_LEFT_KNOB]/100.0; // Derivitive feedback term

    // Reference adjustment
    const float Nbar = kP; //0.1118;
    // ---------------------------------------

    float pitch_angle_deg = 0.0;
    float roll_angle_deg = 0;
    float pitch_rate_deg = 0.0;
    float roll_rate_deg = 0.0;
    float deltaF_pitch_old;
    float deltaF_roll_old;
    float moment_pitch;
    float moment_roll;
    float pitch_err_rad = 0.0;
    float roll_err_rad = 0.0;
    float scale_val = 1.0; // value for scaling controller feedback at low RPM
    // float pitch_rate_err = 0.0;
    // float roll_rate_err = 0.0;

    // Read primary throttle value from left stick
    float input_left_throttle = input_radio_values[INDEX_LEFT_STICK];

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
                motor_control_vector[0] = thrustToMotorValNonlinear(0, input_left_throttle);
                motor_control_vector[2] = thrustToMotorValNonlinear(0, input_left_throttle);
                motor_control_vector[1] = thrustToMotorValNonlinear(0, input_left_throttle);
                motor_control_vector[3] = thrustToMotorValNonlinear(0, input_left_throttle);
            }
            else
            {
                pitch_angle_deg = euler_angles[2];
                roll_angle_deg = euler_angles[1];
                pitch_rate_deg = g[2]; // Through trial and error
                roll_rate_deg = g[1];  // Through trial and error
                // TODO: move each control approach into its own function
                // --- LQR Controller design ---
                // u = Nbar*r - K*x
                // deltaF_pitch = (radioRecieverVals[INDEX_RIGHT_STICK_UPDOWN]*Nbar - pitch*kP)*DEG2RAD + (0.0f*Nbar - pitch_rate*kD)*DEG2RAD;
                // deltaF_roll = (radioRecieverVals[INDEX_RIGHT_STICK_LEFTRIGHT]*Nbar - roll*kP)*DEG2RAD + (0.0f*Nbar - roll_rate*kD)*DEG2RAD;
                // ------------------------------

                // ------------------------------

                // --- PD Controller design ---
                // deltaF_pitch_old = pitch_err*kP + (0.0f - pitch_rate)*DEG2RAD*kD;
                // deltaF_roll_old = roll_err*kP + (0.0f - roll_rate)*DEG2RAD*kD;

                // --- PID Controller design w/ integrator limit ---
                // scale_val = min(1.0, (input_left_throttle / 100.0) * (input_left_throttle / 100.0));
                pitch_err_rad = (-input_radio_values[INDEX_RIGHT_STICK_UPDOWN] - pitch_angle_deg) * DEG2RAD;
                roll_err_rad = (input_radio_values[INDEX_RIGHT_STICK_LEFTRIGHT] - roll_angle_deg) * DEG2RAD;
                integrated_pitch_err_rad = constrain(integrated_pitch_err_rad * 0.999 + (pitch_err_rad * delta_time), -0.25, 0.25);
                integrated_roll_err_rad = constrain(integrated_roll_err_rad * 0.999 + (roll_err_rad * delta_time), -0.25, 0.25);
                moment_pitch = (pitch_err_rad * kP) + ((0.0f - pitch_rate_deg) * DEG2RAD * kD) + (integrated_pitch_err_rad * kI);
                moment_roll = (roll_err_rad * kP) + ((0.0f - roll_rate_deg) * DEG2RAD) * kD + (integrated_roll_err_rad * kI);

                const float force_pitch = moment_pitch / (4 * 0.0995);
                const float force_roll = moment_roll / (4 * 0.0995);

                // otherwise include pitch and roll force requirements
                motor_control_vector[0] = thrustToMotorValNonlinear(-force_pitch - force_roll, input_left_throttle);
                motor_control_vector[2] = thrustToMotorValNonlinear(force_pitch + force_roll, input_left_throttle);
                motor_control_vector[1] = thrustToMotorValNonlinear(-force_pitch + force_roll, input_left_throttle);
                motor_control_vector[3] = thrustToMotorValNonlinear(force_pitch - force_roll, input_left_throttle);
            }
        }
        else
        {
            Serial.println("-------------- DEAD ------------------");

            // if no radio signal, power off the motors
            motor_control_vector[0] = 0.0;
            motor_control_vector[1] = 0.0;
            motor_control_vector[2] = 0.0;
            motor_control_vector[3] = 0.0;
        }
    }

    if (dumb_counter > 15)
    {

        Serial.print(scale_val);
        Serial.print(" | \t");

        // Serial.print(pitch);
        // Serial.print(F("\t"));
        Serial.print(pitch_err_rad);
        Serial.print(F("\t"));
        Serial.print(integrated_pitch_err_rad);
        Serial.print(F("\t | "));
        // Serial.print(deltaF_pitch);
        // Serial.print(F(" | \t"));

        // Serial.print(roll);
        // Serial.print(F("\t"));
        Serial.print(roll_err_rad);
        Serial.print(F("\t"));
        Serial.print(integrated_roll_err_rad);
        Serial.print(F("\t | "));
        // Serial.print(deltaF_roll);
        // Serial.print(F(" | \t"));

        serialPrintArray4(motor_control_vector);
        Serial.println("");

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

    if (micros() - last_rise_time[INDEX_LEFT_STICK] > TIMEOUT_MICROSECONDS)
    {
        return false;
    }
    else
    {
        return true;
    }
}