
#include <Wire.h>   

void serialPrintArray(volatile float *vec){
  Serial.print(vec[0]); Serial.print("\t");
  Serial.print(vec[1]); Serial.print("\t");
  Serial.print(vec[2]); Serial.print("\n");
}

// ###############################################################################################       
void readAccelData(volatile float *accel_vec) //
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  int16_t tmp[3]; 
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  tmp[0] = (int16_t)((int16_t)(rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  tmp[1] = (int16_t)((int16_t)(rawData[2] << 8) | rawData[3]) ;  
  tmp[2] = (int16_t)((int16_t)(rawData[4] << 8) | rawData[5]) ;  
  
  // Now we'll calculate the accleration value into actual m^2/s      
  accel_vec[0] = (((float)tmp[0]) * accel_res - accelBias[0]) * 9.81f;  
  accel_vec[1] = (((float)tmp[1]) * accel_res - accelBias[1]) * 9.81f;   
  accel_vec[2] = (((float)tmp[2]) * accel_res - accelBias[2]) * 9.81f;  
}

// ###############################################################################################     
void adjustAccelData(volatile float *accel_vec, volatile float  *q){
  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q[1] * q[1];
  float q1q2 = q[1] * q[2];
  float q1q3 = q[1] * q[3];
  float q1q4 = q[1] * q[4];
  float q2q2 = q[2] * q[2];
  float q2q3 = q[2] * q[3];
  float q2q4 = q[2] * q[4];
  float q3q3 = q[3] * q[3];
  float q3q4 = q[3] * q[4];
  float q4q4 = q[4] * q[4];  
    
  float Q_ab[3][3] = {{q1q1-q2q2-q3q3+q4q4, 2*(q1q2+q3q4), 2*(q1q3-q2q4)},
                      {2*(q1q2-q3q4), -q1q1+q2q2-q3q3+q4q4, 2*(q2q3+q1q4)},
                      {2*(q1q3+q2q4), 2*(q2q3-q1q4), -q1q1-q2q2+q3q3+q4q4}};
  
}
// ###############################################################################################       
void readGyroData(volatile float *gyro_vec)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  int16_t tmp[3];
   
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  tmp[0] = (int16_t)((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  tmp[1] = (int16_t)((int16_t)rawData[2] << 8) | rawData[3] ;  
  tmp[2] = (int16_t)((int16_t)rawData[4] << 8) | rawData[5] ; 
  
  // Calculate the gyro value into actual degrees per second
  gyro_vec[0] = (float)tmp[0]*gyro_res;  // get actual gyro value, this depends on scale being set
  gyro_vec[1] = (float)tmp[1]*gyro_res;  
  gyro_vec[2] = (float)tmp[2]*gyro_res;   
}

// ###############################################################################################       
void readMagData(volatile float *mag_vec) //int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  int16_t tmp[3]; 
  
  if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
  uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    tmp[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    tmp[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    tmp[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
   }
  }

  magbias[0] = +609.360; //+666.715; //+470.0;  // User environmental x-axis correction in milliGauss, should be automatically calculated
  magbias[1] = +447.380; //+452.795; //+120.0;  // User environmental x-axis correction in milliGauss
  magbias[2] = -400.095; //-412.265; //+125.0;  // User environmental x-axis correction in milliGauss
  
  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental corrections
  m[0] = (float)tmp[0]*mag_res*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
  m[1] = (float)tmp[1]*mag_res*magCalibration[1] - magbias[1];  
  m[2] = (float)tmp[2]*mag_res*magCalibration[2] - magbias[2];   
}

// ###############################################################################################       
int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

// ###############################################################################################       
// Wire.h read and write protocols
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

// ###############################################################################################       
// Wire.h read and write protocols
uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

// ###############################################################################################       
// Wire.h read and write protocols
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
        Wire.requestFrom(address, count);  // Read bytes from slave register address 
  while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

