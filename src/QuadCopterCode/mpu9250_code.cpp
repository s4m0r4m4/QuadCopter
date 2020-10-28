#include <Arduino.h>
#include <mpu9250_code.h>
#include <mpu9250_constants.h>
#include <mpu9250_sensors.h>
#include <Wire.h>
#include <global_junk.h>

/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.

 Demonstrate basic MPU-9250 functionality including parameterizing the register
 addresses, initializing the sensor,
 getting properly scaled accelerometer, gyroscope, and magnetometer data out.
 Added display functions to
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using
 open source Madgwick and
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the
 Teensy 3.1.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.

 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND

 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library.
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a
 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the
 Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L
 /twi.h utility file.
 */

// Specify sensor full scale = 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
const uint8_t magnetometer_mode = 0x06;

float accel_res, gyro_res, mag_res; // scale resolutions per LSB for the sensors
float magnetometer_calibration[3] = {0, 0, 0},
      magnetometer_bias[3] = {0, 0, 0}; // Factory mag calibration and mag bias
float gyro_bias[3] = {0, 0, 0},
      accel_bias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer

// ################################################################################
void ReadMPU9250(float *a, float *g, float *m)
{

    // If intPin goes high, all data registers have new data
    if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {                                            // On interrupt, check if data ready interrupt
        readAccelData(a, accel_res, accel_bias); // Read the x/y/z adc values
        readGyroData(g, gyro_res);               // Read the x/y/z adc values
    }

    if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01)
    {                                                      // wait for magnetometer data ready bit to be set
        readMagData(m, mag_res, magnetometer_calibration); // Read the x/y/z adc values
    }
    // With these settings the filter is updating at a ~145 Hz rate using the
    // Madgwick scheme and
    // >200 Hz using the Mahony scheme even though the display refreshes at only 2
    // Hz.
    // The filter update rate is determined mostly by the mathematical steps in
    // the respective algorithms,
    // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer
    // ODR:
    // an ODR of 10 Hz for the magnetometer produce the above rates, maximum
    // magnetometer ODR of 100 Hz produces
    // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony
    // schemes, respectively.
    // This is presumably because the magnetometer read takes longer than the gyro
    // or accelerometer reads.
    // This filter update rate should be fast enough to maintain accurate platform
    // orientation for
    // stabilization control of a fast-moving robot or quadcopter. Compare to the
    // update rate of 200 Hz
    // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6
    // DoF and MPU9150 9DoF sensors.
    // The 3.3 V 8 MHz Pro Mini is doing pretty well!
}

// #######################################################################################################
void SetupMPU9250()
{

    uint8_t accel_range = 16; // ACC_FULL_SCALE_16_G;
    int gyro_range = 500;     // GYRO_FULL_SCALE_500_DPS;
    uint8_t mag_bits = 16;
    uint8_t gyro_dlpf = 41;  // 0, 41, 92, 184, 250
    uint8_t accel_dlpf = 41; // 0, 41, 92, 184, 460
    char dummy_str[50];

    Wire.begin();
    //  TWBR = 12;  // 400 kbit/sec I2C speed
    Serial.begin(115200);

    // Set the global variables for accel and gyro ranges
    uint8_t gyro_range_cmd;
    switch (gyro_range)
    {
    case 2000:
        gyro_range_cmd = 0x03;
        gyro_res = 2000.0 / 32768.0;
        break; // gyro_range_cmd = 0x18;

    case 1000:
        gyro_range_cmd = 0x02;
        gyro_res = 1000.0 / 32768.0;
        break; // gyro_range_cmd = 0x10;

    case 500:
        gyro_range_cmd = 0x01;
        gyro_res = 500.0 / 32768.0;
        break; // gyro_range_cmd = 0x08;

    case 250:
        gyro_range_cmd = 0x00;
        gyro_res = 250.0 / 32768.0;
        break;

    default:
        Serial.print(F("[ERROR]: Bad Gyro range command: "));
        Serial.println(gyro_range);
    }
    if (VERBOSE)
    {
        Serial.print(F("[Setup] Setting Gryo command:"));
        sprintf(dummy_str, " %x (hex) =  %d (dec) \n", gyro_range_cmd,
                gyro_range_cmd);
        Serial.print(dummy_str);
    }

    // Set the accelrometer range
    uint8_t accel_range_cmd;
    switch (accel_range)
    {
    case 16:
        accel_range_cmd = 0x03;
        accel_res = (16.0) / 32768.0;
        break; // accel_range = 0x18;

    case 8:
        accel_range_cmd = 0x02;
        accel_res = (8.0) / 32768.0;
        break; // accel_range = 0x10;

    case 4:
        accel_range_cmd = 0x01;
        accel_res = (4.0) / 32768.0;
        break; // accel_range = 0x08;

    case 2:
        accel_range_cmd = 0x00;
        accel_res = (2.0) / 32768.0;
        break;

    default:
        Serial.print(F("[ERROR]: Bad Accelerometer range command"));
        Serial.println(accel_range_cmd);
    }
    if (VERBOSE)
    {
        Serial.print(F("[Setup] Setting Accelerometer command:"));
        sprintf(dummy_str, " %x (hex) =  %d (dec) \n", accel_range_cmd,
                accel_range_cmd);
        Serial.print(dummy_str);
    }

    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    uint8_t mag_resolution_cmd;
    switch (mag_bits)
    {
    case 14:
        mag_res = 10.0 * 4912.0 / 8190.0; // Proper scale to return milliGauss
        mag_resolution_cmd = 0x00;
        break;

    case 16:
        mag_res = 10.0 * 4912.0 / 32760.0; // Proper scale to return milliGauss
        mag_resolution_cmd = 0x01;
        break;

    default:
        Serial.println(F("[ERROR] Bad Mag Resolution command"));
        Serial.println(mag_bits);
    }

    // Read the WHO_AM_I register, this is a good test of communication
    byte c = readByte(MPU9250_ADDRESS,
                      WHO_AM_I_MPU9250); // Read WHO_AM_I register for MPU-9250
    if (VERBOSE)
    {
        Serial.print(F("MPU9250 I AM"));
        Serial.print(c, HEX);
        Serial.print(F(" I should be "));
        Serial.println(WHO_AM_I_EXPECTED, HEX);
    }

    if (c == WHO_AM_I_EXPECTED) // WHO_AM_I should always be 0x71
    {
        if (VERBOSE)
        {
            Serial.println(F("MPU9250 is online..."));
            float SelfTest[6]; // holds results of gyro and accelerometer self test

            MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
            Serial.print(F("x-axis self test: acceleration trim within: "));
            Serial.print(SelfTest[0], 1);
            Serial.println(F("% of factory value"));
            Serial.print(F("y-axis self test: acceleration trim within: "));
            Serial.print(SelfTest[1], 1);
            Serial.println(F("% of factory value"));
            Serial.print(F("z-axis self test: acceleration trim within: "));
            Serial.print(SelfTest[2], 1);
            Serial.println(F("% of factory value"));
            Serial.print(F("Gryo self test trim within % of factory value (X,Y,Z): "));
            Serial.print(SelfTest[3], 1);
            Serial.print(SelfTest[4], 1);
            Serial.print(SelfTest[5], 1);
        }

        calibrateMPU9250(gyro_bias, accel_bias); // Calibrate gyro and accelerometers,
                                                 // load biases in bias registers
        //    Serial.print(F("GYRO BIASES: ")); Serial.print(gyro_bias[0]);
        //    Serial.print(F("\t")); Serial.print(gyro_bias[1]); Serial.print(F("\t"));
        //    Serial.print(gyro_bias[2]); Serial.print(F("\n"));

        initMPU9250(gyro_range_cmd, accel_range_cmd, gyro_dlpf, accel_dlpf);

        // Get magnetometer calibration from AK8963 ROM
        initAK8963(magnetometer_calibration, mag_resolution_cmd);

        if (VERBOSE)
        {
            // Initialize device for active mode read of acclerometer, gyroscope, and temperature
            Serial.println(F("MPU9250 initialized for active data mode...."));

            // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
            byte d = readByte(AK8963_ADDRESS,
                              WHO_AM_I_AK8963); // Read WHO_AM_I register for AK8963

            Serial.print(F("AK8963 I AM "));
            Serial.print(d, HEX);
            Serial.print(F(" I should be "));
            Serial.println(0x48, HEX);

            // Initialize device for active mode read of magnetometer
            Serial.println(F("AK8963 initialized for active data mode...."));

            Serial.print(F("X-Axis sensitivity adjustment value "));
            Serial.println(magnetometer_calibration[0], 2);
            Serial.print(F("Y-Axis sensitivity adjustment value "));
            Serial.println(magnetometer_calibration[1], 2);
            Serial.print(F("Z-Axis sensitivity adjustment value "));
            Serial.println(magnetometer_calibration[2], 2);
        }
    }
    else
    {
        Serial.print(F("Could not connect to MPU9250: 0x"));
        Serial.println(c, HEX);

        // I2Cscan(); // TODO remove this
        //II2C_ClearBus(); // TODO remove this
        //I2Cscan(); // TODO remove this
    }
}

// ###############################################################################################
void initMPU9250(int gyro_range_cmd, int accel_range_cmd, int gyro_dlpf,
                 int accel_dlpf)
{
    // wake up device
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    delay(100);                                   // Wait for all registers to reset

    // get stable time source
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01); // Auto select clock source to
                                                  // be PLL gyroscope reference if
                                                  // ready else
    delay(200);

    // ----------------------------------------------------------------------------------------------
    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
    // respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion
    // update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    //  With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8
    //  kHz, or 1 kHz
    switch (gyro_dlpf)
    {
    case 0:
        writeByte(MPU9250_ADDRESS, CONFIG, GYRO_DLPF_CFG_250HZ);
        break; // doesn't matter, because we're going to disable DLPF
    case 41:
        writeByte(MPU9250_ADDRESS, CONFIG, GYRO_DLPF_CFG_41HZ);
        break;
    case 92:
        writeByte(MPU9250_ADDRESS, CONFIG, GYRO_DLPF_CFG_92HZ);
        break;
    case 184:
        writeByte(MPU9250_ADDRESS, CONFIG, GYRO_DLPF_CFG_184HZ);
        break;
    case 250:
        writeByte(MPU9250_ADDRESS, CONFIG, GYRO_DLPF_CFG_250HZ);
        break;
    default:
        Serial.println(F("Gyro DLPF command not recognized! Using 41 Hz..."));
        writeByte(MPU9250_ADDRESS, CONFIG, GYRO_DLPF_CFG_41HZ);
        break;
    }
    uint8_t accel_bandwidth_cmd;
    switch (accel_dlpf)
    {
    case 0:
        accel_bandwidth_cmd = A_DLPF_CFG_460Hz;
        break; // doesn't matter, because we're going to disable DLPF
    case 41:
        accel_bandwidth_cmd = A_DLPF_CFG_41Hz;
        break;
    case 92:
        accel_bandwidth_cmd = A_DLPF_CFG_92Hz;
        break;
    case 184:
        accel_bandwidth_cmd = A_DLPF_CFG_184Hz;
        break;
    case 460:
        accel_bandwidth_cmd = A_DLPF_CFG_460Hz;
        break;
    default:
        Serial.println(F("Accel DLPF command not recognized! Using 41 Hz..."));
        accel_bandwidth_cmd = A_DLPF_CFG_41Hz;
        break;
    }

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because
    // of the SMPLRT_DIV setting
    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    // Use a 200 Hz rate; a rate higher than the filter update rate
    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x5);

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
    // left-shifted into positions 4:3
    uint8_t c = readByte(MPU9250_ADDRESS,
                         GYRO_CONFIG); // get current GYRO_CONFIG register value
    //  c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x02;               // Clear Fchoice bits [1:0]
    c = c & ~0x18;               // Clear AFS bits [4:3]
    c = c | gyro_range_cmd << 3; // Set gyro range
    if (gyro_dlpf != 0)
    {                  // if we want to enable the gryo Digital Low-Pass Filter
        c = c & ~0x03; // Set Fchoice for the gyro to 11 by writing its inverse to
                       // bits 1:0 of GYRO_CONFIG
    }
    else
    {
        c = c | 0x03; // Set Fchoice for the gyro to 00 by writing its inverse to
                      // bits 1:0 of GYRO_CONFIG
    }
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG,
              c); // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
    c = readByte(MPU9250_ADDRESS,
                 ACCEL_CONFIG);   // get current ACCEL_CONFIG register value
    c = c & ~0xE0;                // Clear self-test bits [7:5]
    c = c & ~0x18;                // Clear AFS bits [4:3]
    c = c | accel_range_cmd << 3; // Setthe range for the accelerometer
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG,
              c); // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by
    // choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    c = readByte(MPU9250_ADDRESS,
                 ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
    //  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    if (accel_dlpf != 0)
    {
        c = c & ~0x00 << 3; // enable the accel digital low-pass filter
    }
    else
    {
        c = c | ~0x01 << 3; // disable the accel digital low-pass filter
    }
    c = c | accel_bandwidth_cmd; // Set accelerometer rate to 1 kHz and bandwidth
                                 // to (X) Hz
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2,
              c); // Write new ACCEL_CONFIG2 register value

    c = readByte(MPU9250_ADDRESS, I2C_MST_CTRL);
    //  c = c & 0xF0; // clear the I2C_MST_CLK bits (master clock speed)
    //  c = c & 0x09; // Set master clock speed to 500kHz (probably not necessary)
    //  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, c);

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
    // until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
    writeByte(MPU9250_ADDRESS, INT_ENABLE,
              0x01); // Enable data ready (bit 0) interrupt
    delay(100);
}

// ###############################################################################################
// Initialize the magnetometer
void initAK8963(float *destination, uint8_t mag_resolution_cmd)
{
    // First extract the factory calibration for each magnetometer axis
    uint8_t rawData[3];                           // x/y/z gyro calibration data stored here
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
    delay(10);
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
    delay(10);
    readBytes(AK8963_ADDRESS, AK8963_ASAX, 3,
              &rawData[0]); // Read the x-, y-, and z-axis calibration values
    destination[0] = (float)(rawData[0] - 128) / 256. +
                     1.; // Return x-axis sensitivity adjustment values, etc.
    destination[1] = (float)(rawData[1] - 128) / 256. + 1.;
    destination[2] = (float)(rawData[2] - 128) / 256. + 1.;
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
    delay(10);
    // Configure the magnetometer for continuous read and highest resolution
    // set mag_resolution_cmd bit 4 to 1 (0) to enable 16 (14) bit resolution in
    // CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8
    // Hz and 0110 for 100 Hz sample rates
    writeByte(AK8963_ADDRESS, AK8963_CNTL,
              mag_resolution_cmd << 4 |
                  magnetometer_mode); // Set magnetometer data resolution and sample ODR
    delay(10);
}

// ###############################################################################################
// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into
// accelerometer and gyro bias registers.
void calibrateMPU9250(float *dest1, float *dest2)
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1,
              0x80); // Write a one to bit 7 reset bit; toggle reset device
    delay(100);

    // get stable time source; Auto select clock source to be PLL gyroscope
    // reference if ready
    // else use the internal oscillator, bits 2:0 = 001
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
    delay(200);

    // Configure device for bias calculation
    writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
    writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
    writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
    writeByte(MPU9250_ADDRESS, USER_CTRL,
              0x00);                             // Disable FIFO and I2C master modes
    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C); // Reset FIFO and DMP
    delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00); // Set gyro full-scale to 250
                                                   // degrees per second, maximum
                                                   // sensitivity
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG,
              0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t gyrosensitivity = 131;    // = 131 LSB/degrees/sec
    uint16_t accelsensitivity = 16384; // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40); // Enable FIFO
    writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);   // Enable gyro and accelerometer
    // sensors for FIFO  (max size 512
    // bytes in MPU-9150)
    delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    writeByte(MPU9250_ADDRESS, FIFO_EN,
              0x00); // Disable gyro and accelerometer sensors for FIFO
    readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2,
              &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count =
        fifo_count /
        12; // How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++)
    {
        int16_t accel_tmp[3] = {0, 0, 0}, gyro_tmp[3] = {0, 0, 0};
        readBytes(MPU9250_ADDRESS, FIFO_R_W, 12,
                  &data[0]); // read data for averaging
        accel_tmp[0] = (int16_t)(
            ((int16_t)data[0] << 8) |
            data[1]); // Form signed 16-bit integer for each sample in FIFO
        accel_tmp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
        accel_tmp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
        gyro_tmp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
        gyro_tmp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
        gyro_tmp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

        accel_bias[0] += (int32_t)accel_tmp[0]; // Sum individual signed 16-bit
                                                // biases to get accumulated signed
                                                // 32-bit biases
        accel_bias[1] += (int32_t)accel_tmp[1];
        accel_bias[2] += (int32_t)accel_tmp[2];
        gyro_bias[0] += (int32_t)gyro_tmp[0];
        gyro_bias[1] += (int32_t)gyro_tmp[1];
        gyro_bias[2] += (int32_t)gyro_tmp[2];
    }
    accel_bias[0] /=
        (int32_t)packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t)packet_count;
    accel_bias[2] /= (int32_t)packet_count;
    gyro_bias[0] /= (int32_t)packet_count;
    gyro_bias[1] /= (int32_t)packet_count;
    gyro_bias[2] /= (int32_t)packet_count;

    if (accel_bias[2] > 0L)
    {
        accel_bias[2] -= (int32_t)accelsensitivity;
    } // Remove gravity from the z-axis accelerometer bias calculation
    else
    {
        accel_bias[2] += (int32_t)accelsensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers,
    // which are reset to zero upon device startup
    data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per
                                               // deg/s to conform to expected
                                               // bias input format
    data[1] = (-gyro_bias[0] / 4) & 0xFF;      // Biases are additive, so change sign
                                               // on calculated average gyro biases
    data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4) & 0xFF;
    data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4) & 0xFF;

    // Push gyro biases to hardware registers
    writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
    writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
    writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
    writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
    writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
    writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

    // Output scaled gyro biases for display in the main program
    dest1[0] = (float)gyro_bias[0] / (float)gyrosensitivity;
    dest1[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
    dest1[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer
    // bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer
    // biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved
    // since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input
    // as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {
        0, 0, 0}; // A place to hold the factory accelerometer trim biases
    readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2,
              &data[0]); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
    readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
    readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int32_t)(((int16_t)data[0] << 8) | data[1]);

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of
                         // lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {
        0, 0,
        0}; // Define array to hold mask bit for each accelerometer bias axis

    for (ii = 0; ii < 3; ii++)
    {
        if ((accel_bias_reg[ii] & mask))
            mask_bit[ii] = 0x01; // If temperature compensation bit is set, record
                                 // that fact in mask_bit
                                 //    Serial.print(F("\n Factory Accel Bias = "));
                                 //    Serial.print(accel_bias_reg[ii],DEC);
                                 //    Serial.print(F("\n NEW     Accel Bias = "));
                                 //    Serial.print(accel_bias[ii],DEC);
    }                            // Serial.print(F("\n "));

    // Construct total accelerometer bias, including calculated average
    // accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged
                                              // accelerometer bias scaled to 2048
                                              // LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0]) & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when
                                     // writing back to accelerometer bias
                                     // registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1]) & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when
                                     // writing back to accelerometer bias
                                     // registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2]) & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when
                                     // writing back to accelerometer bias
                                     // registers

    // Apparently this is not working for the acceleration biases in the MPU-9250
    // (TODO)
    // Are we handling the temperature correction bit properly?
    // Push accelerometer biases to hardware registers
    // writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
    // writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
    // writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
    // writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
    // writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
    // writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

    // Output scaled accelerometer biases for display in the main program
    dest2[0] = (float)accel_bias[0] / (float)accelsensitivity; // accelsensitivity
                                                               // is in LSB/g, so
                                                               // the bias is in
                                                               // g's
    dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
    dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
}

// ###############################################################################################
// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float *destination) // Should return percent deviation from
                                         // factory trim values, +/- 14 or less
                                         // deviation is a pass
{
    uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
    uint8_t selfTest[6];
    int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
    float factoryTrim[6];
    uint8_t FS = 0;

    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
    writeByte(MPU9250_ADDRESS, CONFIG,
              0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG,
              1 << FS); // Set full scale range for the gyro to 250 dps
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2,
              0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG,
              1 << FS); // Set full scale range for the accelerometer to 2 g

    for (int ii = 0; ii < 200;
         ii++)
    { // get average current values of gyro and acclerometer

        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6,
                  &rawData[0]); // Read the six raw data registers into data array
        aAvg[0] += (int16_t)(
            ((int16_t)rawData[0] << 8) |
            rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
        aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw
                                                                 // data registers
                                                                 // sequentially
                                                                 // into data array
        gAvg[0] += (int16_t)(
            ((int16_t)rawData[0] << 8) |
            rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
        gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
    }

    for (int ii = 0; ii < 3; ii++)
    { // Get average of 200 values and store as
        // average current readings
        aAvg[ii] /= 200;
        gAvg[ii] /= 200;
    }

    // Configure the accelerometer for self-test
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all
                                                    // three axes and set
                                                    // accelerometer range to +/-
                                                    // 2 g
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0xE0);  // Enable self test on all
                                                    // three axes and set gyro
                                                    // range to +/- 250 degrees/s
    delay(25);                                      // Delay a while to let the device stabilize

    for (int ii = 0; ii < 200;
         ii++)
    { // get average self-test values of gyro and acclerometer

        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6,
                  &rawData[0]); // Read the six raw data registers into data array
        aSTAvg[0] += (int16_t)(
            ((int16_t)rawData[0] << 8) |
            rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
        aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw
                                                                 // data registers
                                                                 // sequentially
                                                                 // into data array
        gSTAvg[0] += (int16_t)(
            ((int16_t)rawData[0] << 8) |
            rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
        gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
    }

    for (int ii = 0; ii < 3; ii++)
    { // Get average of 200 values and store as
        // average self-test readings
        aSTAvg[ii] /= 200;
        gSTAvg[ii] /= 200;
    }

    // Configure the gyro and accelerometer for normal operation
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
    delay(25); // Delay a while to let the device stabilize

    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    selfTest[0] = readByte(MPU9250_ADDRESS,
                           SELF_TEST_X_ACCEL); // X-axis accel self-test results
    selfTest[1] = readByte(MPU9250_ADDRESS,
                           SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
    selfTest[2] = readByte(MPU9250_ADDRESS,
                           SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
    selfTest[3] = readByte(MPU9250_ADDRESS,
                           SELF_TEST_X_GYRO); // X-axis gyro self-test results
    selfTest[4] = readByte(MPU9250_ADDRESS,
                           SELF_TEST_Y_GYRO); // Y-axis gyro self-test results
    selfTest[5] = readByte(MPU9250_ADDRESS,
                           SELF_TEST_Z_GYRO); // Z-axis gyro self-test results

    // Retrieve factory self-test value from self-test code reads
    factoryTrim[0] = (float)(2620 / 1 << FS) *
                     (pow(1.01, ((float)selfTest[0] -
                                 1.0))); // FT[Xa] factory trim calculation
    factoryTrim[1] = (float)(2620 / 1 << FS) *
                     (pow(1.01, ((float)selfTest[1] -
                                 1.0))); // FT[Ya] factory trim calculation
    factoryTrim[2] = (float)(2620 / 1 << FS) *
                     (pow(1.01, ((float)selfTest[2] -
                                 1.0))); // FT[Za] factory trim calculation
    factoryTrim[3] = (float)(2620 / 1 << FS) *
                     (pow(1.01, ((float)selfTest[3] -
                                 1.0))); // FT[Xg] factory trim calculation
    factoryTrim[4] = (float)(2620 / 1 << FS) *
                     (pow(1.01, ((float)selfTest[4] -
                                 1.0))); // FT[Yg] factory trim calculation
    factoryTrim[5] = (float)(2620 / 1 << FS) *
                     (pow(1.01, ((float)selfTest[5] -
                                 1.0))); // FT[Zg] factory trim calculation

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of
    // the Self-Test Response
    // To get percent, must multiply by 100
    for (int i = 0; i < 3; i++)
    {
        destination[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) /
                         factoryTrim[i]; // Report percent differences
        destination[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) /
                             factoryTrim[i + 3]; // Report percent differences
    }
}

/**************************************************************
 * Function: I2Cscan
**************************************************************/
void I2Cscan()
{
    // scan for i2c devices
    byte error;
    byte address;
    int nDevices;

    nDevices = 0;

    Serial.println(F("Scanning..."));
    for (address = 1; address < 127; address++)
    {
        Serial.print(F("Scanning: "));
        Serial.println(address);
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.begin();
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print(F("I2C device found at address 0x"));
            if (address < 16)
                Serial.print(F("0"));
            Serial.print(address, HEX);
            Serial.println(F("  !"));

            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print(F("Unknow error at address 0x"));
            if (address < 16)
                Serial.print(F("0"));
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println(F("No I2C devices found\n"));
    else
        Serial.println(F("done\n"));
}

// // I2C recovery ========================================
// #define SDAPIN A4
// #define CLKPIN A5
// void I2Crecovery() {
//   Serial.println(F("Starting I2C bus recovery"));
//   delay(2000);
//   // try i2c bus recovery at 100kHz = 5uS high, 5uS low
//   pinMode(SDAPIN, OUTPUT); // keeping SDA high during recovery
//   digitalWrite(SDAPIN, HIGH);
//   pinMode(CLKPIN, OUTPUT);
//   for (int i = 0; i < 10; i++) { // 9nth cycle acts as NACK
//     digitalWrite(CLKPIN, HIGH);
//     delayMicroseconds(5);
//     digitalWrite(CLKPIN, LOW);
//     delayMicroseconds(5);
//   }
//
//   // a STOP signal (SDA from low to high while CLK is high)
//   digitalWrite(SDAPIN, LOW);
//   delayMicroseconds(5);
//   digitalWrite(CLKPIN, HIGH);
//   delayMicroseconds(2);
//   digitalWrite(SDAPIN, HIGH);
//   delayMicroseconds(2);
//   // bus status is now : FREE
//
//   Serial.println(F("bus recovery done, starting scan in 2 secs"));
//   // return to power up mode
//   pinMode(SDAPIN, INPUT);
//   pinMode(CLKPIN, INPUT);
//   delay(2000);
//   // pins + begin advised in https://github.com/esp8266/Arduino/issues/452
//   Wire.pins(SDAPIN,
//             CLKPIN); // this changes default values for sda and clock as well
//   Wire.begin(SDAPIN, CLKPIN);
//   // only pins: no signal on clk and sda
//   // only begin: no signal on clk, no signal on sda
// }

// I2C recovery ========================================
// #define SDA A4
// #define SCL A5

// int I2C_ClearBus()
// {
// #if defined(TWCR) && defined(TWEN)
//     TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
// #endif

//     pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
//     pinMode(SCL, INPUT_PULLUP);

//     delay(2500); // Wait 2.5 secs. This is strictly only necessary on the first power
//     // up of the DS3231 module to allow it to initialize properly,
//     // but is also assists in reliable programming of FioV3 boards as it gives the
//     // IDE a chance to start uploaded the program
//     // before existing sketch confuses the IDE by sending Serial data.

//     boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
//     if (SCL_LOW)
//     {             //If it is held low Arduno cannot become the I2C master.
//         return 1; //I2C bus error. Could not clear SCL clock line held low
//     }

//     boolean SDA_LOW = (digitalRead(SDA) == LOW); // vi. Check SDA input.
//     int clockCount = 20;                         // > 2x9 clock

//     while (SDA_LOW && (clockCount > 0))
//     { //  vii. If SDA is Low,
//         clockCount--;
//         // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
//         pinMode(SCL, INPUT);        // release SCL pullup so that when made output it will be LOW
//         pinMode(SCL, OUTPUT);       // then clock SCL Low
//         delayMicroseconds(10);      //  for >5uS
//         pinMode(SCL, INPUT);        // release SCL LOW
//         pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
//         // do not force high as slave may be holding it low for clock stretching.
//         delayMicroseconds(10); //  for >5uS
//         // The >5uS is so that even the slowest I2C devices are handled.
//         SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
//         int counter = 20;
//         while (SCL_LOW && (counter > 0))
//         { //  loop waiting for SCL to become High only wait 2sec.
//             counter--;
//             delay(100);
//             SCL_LOW = (digitalRead(SCL) == LOW);
//         }
//         if (SCL_LOW)
//         {             // still low after 2 sec error
//             return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
//         }
//         SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
//     }
//     if (SDA_LOW)
//     {             // still low
//         return 3; // I2C bus error. Could not clear. SDA data line held low
//     }

//     // else pull SDA line low for Start or Repeated Start
//     pinMode(SDA, INPUT);  // remove pullup.
//     pinMode(SDA, OUTPUT); // and then make it LOW i.e. send an I2C Start or Repeated start control.
//     // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
//     /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
//     delayMicroseconds(10);      // wait >5uS
//     pinMode(SDA, INPUT);        // remove output low
//     pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
//     delayMicroseconds(10);      // x. wait >5uS
//     pinMode(SDA, INPUT);        // and reset pins as tri-state inputs which is the default state on reset
//     pinMode(SCL, INPUT);
//     return 0; // all ok
// }
