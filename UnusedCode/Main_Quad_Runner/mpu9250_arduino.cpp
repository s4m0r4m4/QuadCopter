#include <Wire.h>
#include <TimerOne.h>

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in 
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
//
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C<<1
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E


// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}

/* **********************************************
 * Write a byte (Data) in device (Address) at register (Register)
 ********************************************** */
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}


/* **********************************************
 * Setup/Configure the MPU9250
 ********************************************** */
void setup_mpu9250(int accel_range, int gyro_range, float *mpu9250_scaling)//, float *mpu9250_offset)
{
  bool verbose = false;
  
  char dummy_str[50];
  uint8_t gyro_command;
  float gyro_res;
  uint8_t accel_command;
  float accel_res;
  
  // Arduino initializations
  Wire.begin();
  Serial.begin(115200);
  
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);
  
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 26, 0x06); 
  
  // Configure gyroscope range    
  //#define    GYRO_FULL_SCALE_250_DPS    0x00  
  //#define    GYRO_FULL_SCALE_500_DPS    0x08
  //#define    GYRO_FULL_SCALE_1000_DPS   0x10
  //#define    GYRO_FULL_SCALE_2000_DPS   0x18  // degree per second
  switch (gyro_range)
  {
    case 2000:
      gyro_command = 0x18;
      gyro_res = 2000.0/32768.0;
      break;
    case 1000:
      gyro_command = 0x10;
      gyro_res = 1000.0/32768.0;
      break;
    case 500:
      gyro_command = 0x08;
      gyro_res = 500.0/32768.0;
      break;
    case 250:
      gyro_command = 0x00;
      gyro_res = 250.0/32768.0;
      break;
  }
  
  if (verbose) {
    Serial.print("[Setup] Setting Gryo command:");
    sprintf(dummy_str, " %x (hex) =  %d (dec) \n", gyro_command, gyro_command);
    Serial.print(dummy_str);
  }

  I2CwriteByte(MPU9250_ADDRESS, 27, gyro_command);

  // Configure accelerometers range
  //#define    ACC_FULL_SCALE_2_G        0x00  
  //#define    ACC_FULL_SCALE_4_G        0x08
  //#define    ACC_FULL_SCALE_8_G        0x10
  //#define    ACC_FULL_SCALE_16_G       0x18  //g's (9.81 m/s^2)
  switch (accel_range)
  {
    case 16:
      accel_command = 0x18;
      accel_res = (16.0 * 9.81)/32768.0;
      break;
    case 8:
      accel_command = 0x10;
      accel_res = (8.0 * 9.81)/32768.0;
      break;
    case 4:
      accel_command = 0x08;
      accel_res = (4.0 * 9.81)/32768.0;
      break;
    case 2:
      accel_command = 0x00;
      accel_res = (2.0 * 9.81)/32768.0;
      break;
  }
  if (verbose) {
    Serial.print("[Setup] Setting Accelerometer command:");
    sprintf(dummy_str, " %x (hex) =  %d (dec) \n", accel_command, accel_command);
    Serial.print(dummy_str);
  }
  I2CwriteByte(MPU9250_ADDRESS, 28, accel_command);

  // Set bypass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);
  
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16); 
  
  mpu9250_scaling[0] = accel_res;
  mpu9250_scaling[1] = gyro_res;
  mpu9250_scaling[2] = 4800.0/32768.0;

  //read_mpu9250(volatile float *a, volatile float *g, volatile float *m, float *mpu9250_scaling)
  
  // Store initial time
  //ti=millis();
}



// Counter
//long int cpt=0;

//void callback()
//{ 
//  intFlag=true;
//  digitalWrite(13, digitalRead(13) ^ 1);
//}


/* **********************************************
 * Read a,g and m from MPU9250
 ********************************************** */
void read_mpu9250(volatile float *a, volatile float *g, volatile float *m, float *mpu9250_scaling) //volatile bool *intFlag, 
{
  // ____________________________________
  // :::  accelerometer and gyroscope ::: 

  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  
  // Create 16 bits values from 8 bits data  
  // Accelerometer
  a[0]=-(Buf[0]<<8 | Buf[1]) * mpu9250_scaling[0];
  a[1]=-(Buf[2]<<8 | Buf[3]) * mpu9250_scaling[0];
  a[2]=(Buf[4]<<8 | Buf[5]) * mpu9250_scaling[0];

//  // Gyroscope
  g[0]=-(Buf[8]<<8 | Buf[9]) * mpu9250_scaling[1];
  g[1]=-(Buf[10]<<8 | Buf[11]) * mpu9250_scaling[1];
  g[2]=(Buf[12]<<8 | Buf[13]) * mpu9250_scaling[1];
 
  // _____________________
  // :::  Magnetometer ::: 
  
  // Read register Status 1 and wait for the DRDY: Data Ready  
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t Mag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);  

  // Create 16 bits values from 8 bits data  
  // Magnetometer
  m[0]=-(Mag[3]<<8 | Mag[2]) * mpu9250_scaling[2];
  m[1]=-(Mag[1]<<8 | Mag[0]) * mpu9250_scaling[2];
  m[2]=-(Mag[5]<<8 | Mag[4]) * mpu9250_scaling[2];

}


/* **********************************************
 * Displays a,g and m to Serial
 ********************************************** */
void display_mpu9250_data(float *a, float *g, float *m)
{
  bool verbose = true;
  // Accelerometer
  if(verbose){
    Serial.print (a[0],3); 
    Serial.print ("\t");
    Serial.print (a[1],3);
    Serial.print ("\t");
    Serial.print (a[2],3);  
    Serial.print ("\t");
  }
  
  // Gyroscope
  if(verbose){
    Serial.print (g[0],2); 
    Serial.print ("\t");
    Serial.print (g[1],2);
    Serial.print ("\t");
    Serial.print (g[2],2);  
    Serial.print ("\t");
  }    
  
  // Magnetometer
  if(verbose){
    Serial.print (m[0]+200,1); 
    Serial.print ("\t");
    Serial.print (m[1]-70,1);
    Serial.print ("\t");
    Serial.print (m[2]-700,1);  
    Serial.print ("\t");  
  
  // End of line
  Serial.println("");  
  }  
  
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
//void calibrateMPU9250(float * dest1, float * dest2)
//{  
//  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
//  uint16_t ii, packet_count, fifo_count;
//  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
//  
//// reset device, reset all registers, clear gyro and accelerometer bias registers
//  I2CwriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
//  wait(0.1);  
//   
//// get stable time source
//// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
//  I2CwriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  
//  I2CwriteByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00); 
//  wait(0.2);
//  
//// Configure device for bias calculation
//  I2CwriteByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
//  I2CwriteByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
//  I2CwriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
//  I2CwriteByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
//  I2CwriteByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
//  I2CwriteByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
//  wait(0.015);
//  
//// Configure MPU9250 gyro and accelerometer for bias calculation
//  I2CwriteByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
//  I2CwriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
//  I2CwriteByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
//  I2CwriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
// 
//  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
//  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g
//
//// Configure FIFO to capture accelerometer and gyro data for bias calculation
//  I2CwriteByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
//  I2CwriteByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
//  wait(0.04); // accumulate 40 samples in 80 milliseconds = 480 bytes
//
//// At end of sample accumulation, turn off FIFO sensor read
//  I2CwriteByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
//  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
//  fifo_count = ((uint16_t)data[0] << 8) | data[1];
//  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
//
//  for (ii = 0; ii < packet_count; ii++) {
//    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
//    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
//    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
//    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
//    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
//    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
//    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
//    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
//    
//    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
//    accel_bias[1] += (int32_t) accel_temp[1];
//    accel_bias[2] += (int32_t) accel_temp[2];
//    gyro_bias[0]  += (int32_t) gyro_temp[0];
//    gyro_bias[1]  += (int32_t) gyro_temp[1];
//    gyro_bias[2]  += (int32_t) gyro_temp[2];
//            
//}
//    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
//    accel_bias[1] /= (int32_t) packet_count;
//    accel_bias[2] /= (int32_t) packet_count;
//    gyro_bias[0]  /= (int32_t) packet_count;
//    gyro_bias[1]  /= (int32_t) packet_count;
//    gyro_bias[2]  /= (int32_t) packet_count;
//    
//  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
//  else {accel_bias[2] += (int32_t) accelsensitivity;}
// 
//// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
//  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
//  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
//  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
//  data[3] = (-gyro_bias[1]/4)       & 0xFF;
//  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
//  data[5] = (-gyro_bias[2]/4)       & 0xFF;
//
///// Push gyro biases to hardware registers
///*  I2CwriteByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
//  I2CwriteByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
//  I2CwriteByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
//  I2CwriteByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
//  I2CwriteByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
//  I2CwriteByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
//*/
//  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
//  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
//  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;
//
//// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
//// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
//// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
//// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
//// the accelerometer biases calculated above must be divided by 8.
//
//  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
//  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
//  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
//  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
//  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
//  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
//  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
//  
//  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
//  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
//  
//  for(ii = 0; ii < 3; ii++) {
//    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
//  }
//
//  // Construct total accelerometer bias, including calculated average accelerometer bias from above
//  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
//  accel_bias_reg[1] -= (accel_bias[1]/8);
//  accel_bias_reg[2] -= (accel_bias[2]/8);
// 
//  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
//  data[1] = (accel_bias_reg[0])      & 0xFF;
//  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
//  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
//  data[3] = (accel_bias_reg[1])      & 0xFF;
//  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
//  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
//  data[5] = (accel_bias_reg[2])      & 0xFF;
//  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
//
//// Apparently this is not working for the acceleration biases in the MPU-9250
//// Are we handling the temperature correction bit properly?
//// Push accelerometer biases to hardware registers
///*  I2CwriteByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
//  I2CwriteByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
//  I2CwriteByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
//  I2CwriteByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
//  I2CwriteByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
//  I2CwriteByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
//*/
//// Output scaled accelerometer biases for manual subtraction in the main program
//   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
//   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
//   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
//}







