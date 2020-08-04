#include <Arduino.h>

void ReadMPU9250(float *a, float *g, float *m);

void SetupMPU9250();

void I2Cscan();

void MPU9250SelfTest(float *destination);
void calibrateMPU9250(float *dest1, float *dest2);
void initMPU9250(int gyro_range_cmd, int accel_range_cmd, int gyro_dlpf,
                 int accel_dlpf);

void initAK8963(float *destination, uint8_t mag_resolution_cmd);
int I2C_ClearBus();