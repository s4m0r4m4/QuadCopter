#include <Arduino.h>
#include <Wire.h>
#include <mpu9250_constants.h>
#include <mpu9250_sensors.h>

/**************************************************************
 * Function: readAccelData
**************************************************************/
void readAccelData(float *accel_vec, float accel_res, float *accel_bias_vec) //
{

    uint8_t rawData[6]; // x/y/z accel register data stored here
    int16_t tmp[3];
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);    // Read the six raw data registers into data array
    tmp[0] = (int16_t)((int16_t)(rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
    tmp[1] = (int16_t)((int16_t)(rawData[2] << 8) | rawData[3]);
    tmp[2] = (int16_t)((int16_t)(rawData[4] << 8) | rawData[5]);

    // Now we'll calculate the accleration value into actual m^2/s
    accel_vec[0] = (((float)tmp[0]) * accel_res - accel_bias_vec[0]) * 9.81f;
    accel_vec[1] = (((float)tmp[1]) * accel_res - accel_bias_vec[1]) * 9.81f;
    accel_vec[2] = (((float)tmp[2]) * accel_res - accel_bias_vec[2]) * 9.81f;
}

// ###############################################################################################
void adjustAccelData(float *accel_vec, float *quat)
{
    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = quat[0] * quat[0];
    float q1q2 = quat[0] * quat[1];
    float q1q3 = quat[0] * quat[2];
    // float q1q4 = quat[0] * quat[3];
    float q2q2 = quat[1] * quat[1];
    // float q2q3 = quat[1] * quat[2];
    float q2q4 = quat[1] * quat[3];
    float q3q3 = quat[2] * quat[2];
    float q3q4 = quat[2] * quat[3];
    float q4q4 = quat[3] * quat[3];

    // Flipping q0 and q4
    //  float q1q1 = q[1] * q[1];
    //  float q1q2 = q[1] * q[2];
    //  float q1q3 = q[1] * q[3];
    //  float q1q4 = q[1] * q[0];
    //  float q2q2 = q[2] * q[2];
    //  float q2q3 = q[2] * q[3];
    //  float q2q4 = q[2] * q[0];
    //  float q3q3 = q[3] * q[3];
    //  float q3q4 = q[3] * q[0];
    //  float q4q4 = q[0] * q[0];

    //    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    //    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    //    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    //    pitch *= 180.0f / PI;
    //    yaw   *= 180.0f / PI;
    //    yaw   -= 12.2; // Declination at Burbank, California is 12.2 degrees
    //    roll  *= 180.0f / PI;
    //      Serial.print(yaw, 2);  Serial.print("\t ");
    //      Serial.print(pitch, 2);  Serial.print("\t ");
    //      Serial.print(roll, 2);  Serial.println("\t ");

    //  float Q_ab[3][3] = {{q1q1-q2q2-q3q3+q4q4, 2*(q1q2+q3q4), 2*(q1q3-q2q4)},
    //                      {2*(q1q2-q3q4), -q1q1+q2q2-q3q3+q4q4, 2*(q2q3+q1q4)},
    //                      {2*(q1q3+q2q4), 2*(q2q3-q1q4), -q1q1-q2q2+q3q3+q4q4}};

    //  float Q_ab[3] = {q1q1-q2q2-q3q3+q4q4, 2*(q1q2-q3q4), 2*(q1q3+q2q4)};
    float Q_ab[3] = {q1q1 - q2q2 - q3q3 + q4q4, 2 * (q1q2 + q3q4), 2 * (q1q3 - q2q4)};
    //  float q_star[4] = {q[0], -q[1], -q[2], -q[3]};

    //  Serial.println(sqrt((2*(q1q3-q2q4))*(2*(q1q3-q2q4)) + (2*(q2q3+q1q4))*(2*(q2q3+q1q4)) + (-q1q1-q2q2+q3q3+q4q4)*(-q1q1-q2q2+q3q3+q4q4)));

    float new_g[3];
    for (int i = 0; i < 3; i++)
    {
        new_g[2 - i] = Q_ab[i] * -9.81; //matrix multiplication, simplified for a single-valued g-vector of constant value
    }
    accel_vec[0] = accel_vec[0] - new_g[0];
    accel_vec[1] = accel_vec[1] + new_g[1];
    accel_vec[2] = accel_vec[2] + new_g[2];

    //  serialPrintArray(accel_vec);
    //  serialPrintArray(new_g);
    //  serialPrintArrayLn(new_accel_vec);
}

/**************************************************************
 * Function: readGyroData
**************************************************************/
void readGyroData(float *gyro_vec, float gyro_res)
{
    uint8_t rawData[6]; // x/y/z gyro register data stored here
    int16_t tmp[3];

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);   // Read the six raw data registers sequentially into data array
    tmp[0] = (int16_t)((int16_t)rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a signed 16-bit value
    tmp[1] = (int16_t)((int16_t)rawData[2] << 8) | rawData[3];
    tmp[2] = (int16_t)((int16_t)rawData[4] << 8) | rawData[5];

    // Calculate the gyro value into actual degrees per second
    gyro_vec[0] = (float)tmp[0] * gyro_res; // get actual gyro value, this depends on scale being set
    gyro_vec[1] = (float)tmp[1] * gyro_res;
    gyro_vec[2] = (float)tmp[2] * gyro_res;
}

// ###############################################################################################
void readMagData(float *mag_vec, float mag_res, float *magCalibration) //int16_t * destination)
{
    uint8_t rawData[7]; // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    int16_t tmp[3];
    float magbias[3] = {+609.360, +447.380, -400.095}; // User environmental x, y, and z-axis correction in milliGauss
    //  237.5 +N // from http://www.ngdc.noaa.gov/geomag-web/#igrfwmm
    //  51.672 + E
    // 404.095 + Up vertical

    if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01)
    {                                                             // wait for magnetometer data ready bit to be set
        readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]); // Read the six raw data and ST2 registers sequentially into data array
        uint8_t c = rawData[6];                                   // End data read by reading ST2 register
        if (!(c & 0x08))
        {                                                     // Check if magnetic sensor overflow set, if not then report data
            tmp[0] = ((int16_t)rawData[1] << 8) | rawData[0]; // Turn the MSB and LSB into a signed 16-bit value
            tmp[1] = ((int16_t)rawData[3] << 8) | rawData[2]; // Data stored as little Endian
            tmp[2] = ((int16_t)rawData[5] << 8) | rawData[4];
        }
    }

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mag_vec[0] = (float)tmp[0] * mag_res * magCalibration[0] - magbias[0]; // get actual magnetometer value, this depends on scale being set
    mag_vec[1] = (float)tmp[1] * mag_res * magCalibration[1] - magbias[1];
    mag_vec[2] = (float)tmp[2] * mag_res * magCalibration[2] - magbias[2];
}

// ###############################################################################################
// Wire.h read and write protocols
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address); // Initialize the Tx buffer
    Wire.write(subAddress);          // Put slave register address in Tx buffer
    Wire.write(data);                // Put data in Tx buffer
    Wire.endTransmission();          // Send the Tx buffer
}

// ###############################################################################################
// Wire.h read and write protocols
uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data;                          // `data` will store the register data
    Wire.beginTransmission(address);       // Initialize the Tx buffer
    Wire.write(subAddress);                // Put slave register address in Tx buffer
    Wire.endTransmission(false);           // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (uint8_t)1); // Read one byte from slave register address
    data = Wire.read();                    // Fill Rx buffer with result
    return data;                           // Return data read from slave register
}

// ###############################################################################################
// Wire.h read and write protocols
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
    Wire.beginTransmission(address); // Initialize the Tx buffer
    Wire.write(subAddress);          // Put slave register address in Tx buffer
    Wire.endTransmission(false);     // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, count); // Read bytes from slave register address
    while (Wire.available())
    {
        dest[i++] = Wire.read();
    } // Put read results in the Rx buffer
}
