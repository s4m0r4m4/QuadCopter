#include <Arduino.h>
#include <math.h>
#include <state_estimator.h>
#include <serial_printing.h>
#include <mpu9250_sensors.h>
#include <quadcopter_constants.h>

/**************************************************************
 * Variables
**************************************************************/
float eInt[3] = {0.0f, 0.0f, 0.0f}; // vector to hold integral error for estimator Mahony method
float x[3];                         // Linear position: x, y, z
float v[3];                         // inear velocity: vx, vy, vz

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
// gyroscope measurement error in rads/s (start at 40 deg/s)
const float GyroMeasError =
    PI * (40.0f /
          180.0f);
// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
const float GyroMeasDrift =
    PI *
    (0.0f /
     180.0f);
// compute beta
const float MAGDWICK_BETA = sqrt(3.0f / 4.0f) * GyroMeasError * 1.5;
// compute zeta [NOT CURRENTLY USED], the other free parameter in the Madgwick
// scheme usually set to a small or zero value
const float MAGDWICK_ZETA = sqrt(3.0f / 4.0f) * GyroMeasDrift;

// There is a tradeoff in the beta parameter between accuracy and response
// speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError
// of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a
// stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not
// fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response
// time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the
// I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges,
// usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion
// scheme.

// these are the free parameters in the Mahony filter and fusion scheme, KP_MAHONEY for
// proportional feedback, KI_MAHONEY for integral
#define KP_MAHONEY 25.0f
#define KI_MAHONEY 0.25f

/**************************************************************
 * Function: updateState
**************************************************************/
void updateState(float *a, float *g, float *m, float *q, float delta_time, float *euler_angles)
{

    float pitch, yaw, roll;

    //  MadgwickQuaternionUpdate(a[0]/9.81f, a[1]/9.81f, a[2]/9.81f, g[0]*PI/180.0f, g[1]*PI/180.0f, g[2]*PI/180.0f,  m[1], m[0], m[2]);
    MadgwickQuaternionUpdate(a, g, m, q, delta_time);
    //  MahonyQuaternionUpdate(a[0], a[1], a[2], g[0]*PI/180.0f, g[1]*PI/180.0f, g[2]*PI/180.0f, my, mx, mz);
    // MahonyQuaternionUpdate(a, g, m, q, delta_time);

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
    yaw = atan2(
        2.0f * (q[1] * q[2] + q[0] * q[3]),
        q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll = -atan2(
        -2.0f * (q[0] * q[1] + q[2] * q[3]),
        -1.0f * (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]));

    yaw = yaw / DEG2RAD; // Declination at Burbank, California is 12.2 degrees. Minneapolis is basically 0deg
    pitch = pitch / DEG2RAD;
    roll = roll / DEG2RAD;

    // Set gloabl variables (as degrees)
    euler_angles[0] = yaw;
    euler_angles[1] = roll;
    euler_angles[2] = pitch;

    if ((roll < 5) && (roll > -5) && (pitch < 5) && (pitch > -5))
    {
        digitalWrite(LED_LEVEL_INDICATOR, HIGH);
    }
    else
    {
        digitalWrite(LED_LEVEL_INDICATOR, LOW);
    }
    digitalWrite(LED_LEVEL_INDICATOR, HIGH);

    // float v_avg = 0;
    const float cutoff = 0.1;

    //if (Now>5200000){
    for (int j = 0; j < 3; j++)
    {
        //v_avg = 1.0/2.0*(v[j] + (v[j] + a[j]*delta_time));
        if (abs(a[j]) > cutoff)
        {
            v[j] = v[j] + a[j] * delta_time;
        }
        if (abs(v[j]) > cutoff)
        {
            x[j] = x[j] + v[j] * delta_time;
            //x[j] += v_avg*delta_time;
        }
    }

    //  Serial.print(Now); Serial.print("\t");
    // serialPrintArray(a);
    // serialPrintArray(g);
    // serialPrintArray(m);
    //  serialPrintArray(v);
    //  serialPrintArrayLn(x);
    //  serialPrintArray(q);
    // serialPrintArray(euler_angles);
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
//void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
void MadgwickQuaternionUpdate(float *a, float *g, float *m, float *q, float delta_time)
{
    // TODO: make these consts
    float ax = a[0];
    float ay = a[1];
    float az = a[2];

    float gx = g[0] * PI / 180.0f; // Pass gyro rate as rad/s
    float gy = g[1] * PI / 180.0f;
    float gz = g[2] * PI / 180.0f;

    float mx = m[1]; //switch mx and my
    float my = m[0]; //switch mx and my
    float mz = m[2];

    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f)
        return; // handle NaN
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f)
        return; // handle NaN
    norm = 1.0f / norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
    norm = 1.0f / norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - MAGDWICK_BETA * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - MAGDWICK_BETA * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - MAGDWICK_BETA * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - MAGDWICK_BETA * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * delta_time;
    q2 += qDot2 * delta_time;
    q3 += qDot3 * delta_time;
    q4 += qDot4 * delta_time;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // normalise quaternion
    norm = 1.0f / norm;

    // Store in output quaternion vector
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
// measured ones.

// Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
// the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
// We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
// For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
// in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.

//void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
void MahonyQuaternionUpdate(float *a, float *g, float *m, float *q, float delta_time)
{

    //    MadgwickQuaternionUpdate(-ay, -ax, az, gy*PI/180.0f, gx*PI/180.0f,
    //-gz*PI/180.0f,  mx,  my, mz);
    //  if(passThru)MahonyQuaternionUpdate(-ay, -ax, az, gy*PI/180.0f,
    //gx*PI/180.0f, -gz*PI/180.0f,  mx,  my, mz);

    //    float ax = -a[1];
    //    float ay = -a[0];
    //    float az = a[2];
    //
    //    float gx = g[1]*PI/180.0f; // Pass gyro rate as rad/s
    //    float gy = g[0]*PI/180.0f;
    //    float gz = -g[2]*PI/180.0f;
    //
    //    float mx = m[0];
    //    float my = m[1];
    //    float mz = m[2];

    float ax = a[0];
    float ay = a[1];
    float az = a[2];

    float gx = g[0] * PI / 180.0f; // Pass gyro rate as rad/s
    float gy = g[1] * PI / 180.0f;
    float gz = g[2] * PI / 180.0f;

    float mx = m[1];
    float my = m[0];
    float mz = -m[2];

    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; // short name local variable for readability
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f)
        return;         // handle NaN
    norm = 1.0f / norm; // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f)
        return;         // handle NaN
    norm = 1.0f / norm; // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
    hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

    // Estimated direction of gravity and magnetic field
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    if (KI_MAHONEY > 0.0f)
    {
        eInt[0] += ex; // accumulate integral error
        eInt[1] += ey; // missing delta_time? I guess thats fine as long as its taken into account in K_i
        eInt[2] += ez;
    }
    else
    {
        eInt[0] = 0.0f; // prevent integral wind up
        eInt[1] = 0.0f;
        eInt[2] = 0.0f;
    }

    // Apply feedback terms
    gx = gx + KP_MAHONEY * ex + KI_MAHONEY * eInt[0];
    gy = gy + KP_MAHONEY * ey + KI_MAHONEY * eInt[1];
    gz = gz + KP_MAHONEY * ez + KI_MAHONEY * eInt[2];

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * delta_time);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * delta_time);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * delta_time);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * delta_time);

    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}
