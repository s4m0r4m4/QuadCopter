#include <Wire.h>
#include <TimerOne.h>

#ifndef MPU9250_ADDRESS
#define MPU9250_ADDRESS
#endif


//#define    MAG_ADDRESS                0x0C
#ifndef MAG_ADDRESS
#define MAG_ADDRESS
#endif
//
////#define    GYRO_FULL_SCALE_250_DPS    0x00  
//#ifndef GYRO_FULL_SCALE_250_DPS
//#define GYRO_FULL_SCALE_250_DPS
//#endif
//
////#define    GYRO_FULL_SCALE_500_DPS    0x08
//#ifndef GYRO_FULL_SCALE_500_DPS
//#define GYRO_FULL_SCALE_500_DPS
//#endif
//
////#define    GYRO_FULL_SCALE_1000_DPS   0x10
//#ifndef GYRO_FULL_SCALE_1000_DPS
//#define GYRO_FULL_SCALE_1000_DPS
//#endif
//
////#define    GYRO_FULL_SCALE_2000_DPS   0x18  // degree per second
//#ifndef GYRO_FULL_SCALE_2000_DPS
//#define GYRO_FULL_SCALE_2000_DPS
//#endif
//
////#define    ACC_FULL_SCALE_2_G        0x00  
//#ifndef ACC_FULL_SCALE_2_G
//#define ACC_FULL_SCALE_2_G
//#endif
//
////#define    ACC_FULL_SCALE_4_G        0x08
//#ifndef ACC_FULL_SCALE_4_G 
//#define ACC_FULL_SCALE_4_G 
//#endif
//
////#define    ACC_FULL_SCALE_8_G        0x10
//#ifndef ACC_FULL_SCALE_8_G
//#define ACC_FULL_SCALE_8_G
//#endif
//
////#define    ACC_FULL_SCALE_16_G       0x18  //g's (9.81 m/s^2)
//#ifndef ACC_FULL_SCALE_16_G
//#define ACC_FULL_SCALE_16_G
//#endif

// Define scaling factors for readings from MPU 9250
//float MPU_RANGE_GYRO = 32767.0/2000.0; //deg/sec
//float MPU_RANGE_ACCEL = 32767.0/16.0; //g's
//float MPU_RANGE_MAG = 32767.0/4800.0; //micro-Teslas

//typedef uint8_t uint8_t;

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);


// Initial time
	//long int ti;
	//volatile bool intFlag=false;

// Initializations for MPU 9250 board
void setup_mpu9250(int accel_range, int gyro_range, float *MPU9250_RES);

//// Counter
//long int cpt=0;

//void callback();

// Read data from MPU 9250
void read_mpu9250(volatile float *a, volatile float *g, volatile float *m, float *MPU9250_RES); //volatile bool *intFlag, 


void display_mpu9250_data(float *a, float *g, float *m);









