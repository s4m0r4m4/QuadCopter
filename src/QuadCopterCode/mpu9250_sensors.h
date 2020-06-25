#include <Arduino.h>
#include <Wire.h>
#include <mpu9250_constants.h>

/**************************************************************
 * Function: readAccelData
**************************************************************/
void readAccelData(float *accel_vec, float accel_res, float *accel_bias_vec);

/**************************************************************
 * Function: adjustAcccelData
**************************************************************/
void adjustAccelData(float *accel_vec, float *quat);

/**************************************************************
 * Function: readGyroData
**************************************************************/
void readGyroData(float *gyro_vec, float gyro_res);

/**************************************************************
 * Function: readMagData
**************************************************************/
void readMagData(float *mag_vec, float mag_res, float *magCalibration);

/**************************************************************
 * Function: writeByte
**************************************************************/
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);

/**************************************************************
 * Function: readByte
**************************************************************/
uint8_t readByte(uint8_t address, uint8_t subAddress);

/**************************************************************
 * Function: readBytes
**************************************************************/
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest);