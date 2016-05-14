/* IMU chipset BMX055 Library
 * Copyright (c) 2016 Ioton Technology
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef BMX055_H_
#define BMX055_H_

/* Includes ------------------------------------------------------------------*/
#include "mbed.h"


// Accelerometer registers
#define BMX055_ACC_WHOAMI        0x00   // should return 0xFA
//#define BMX055_ACC_Reserved    0x01
#define BMX055_ACC_D_X_LSB       0x02
#define BMX055_ACC_D_X_MSB       0x03
#define BMX055_ACC_D_Y_LSB       0x04
#define BMX055_ACC_D_Y_MSB       0x05
#define BMX055_ACC_D_Z_LSB       0x06
#define BMX055_ACC_D_Z_MSB       0x07
#define BMX055_ACC_D_TEMP        0x08
#define BMX055_ACC_INT_STATUS_0  0x09
#define BMX055_ACC_INT_STATUS_1  0x0A
#define BMX055_ACC_INT_STATUS_2  0x0B
#define BMX055_ACC_INT_STATUS_3  0x0C
//#define BMX055_ACC_Reserved    0x0D
#define BMX055_ACC_FIFO_STATUS   0x0E
#define BMX055_ACC_PMU_RANGE     0x0F
#define BMX055_ACC_PMU_BW        0x10
#define BMX055_ACC_PMU_LPW       0x11
#define BMX055_ACC_PMU_LOW_POWER 0x12
#define BMX055_ACC_D_HBW         0x13
#define BMX055_ACC_BGW_SOFTRESET 0x14
//#define BMX055_ACC_Reserved    0x15
#define BMX055_ACC_INT_EN_0      0x16
#define BMX055_ACC_INT_EN_1      0x17
#define BMX055_ACC_INT_EN_2      0x18
#define BMX055_ACC_INT_MAP_0     0x19
#define BMX055_ACC_INT_MAP_1     0x1A
#define BMX055_ACC_INT_MAP_2     0x1B
//#define BMX055_ACC_Reserved    0x1C
//#define BMX055_ACC_Reserved    0x1D
#define BMX055_ACC_INT_SRC       0x1E
//#define BMX055_ACC_Reserved    0x1F
#define BMX055_ACC_INT_OUT_CTRL  0x20
#define BMX055_ACC_INT_RST_LATCH 0x21
#define BMX055_ACC_INT_0         0x22
#define BMX055_ACC_INT_1         0x23
#define BMX055_ACC_INT_2         0x24
#define BMX055_ACC_INT_3         0x25
#define BMX055_ACC_INT_4         0x26
#define BMX055_ACC_INT_5         0x27
#define BMX055_ACC_INT_6         0x28
#define BMX055_ACC_INT_7         0x29
#define BMX055_ACC_INT_8         0x2A
#define BMX055_ACC_INT_9         0x2B
#define BMX055_ACC_INT_A         0x2C
#define BMX055_ACC_INT_B         0x2D
#define BMX055_ACC_INT_C         0x2E
#define BMX055_ACC_INT_D         0x2F
#define BMX055_ACC_FIFO_CONFIG_0 0x30
//#define BMX055_ACC_Reserved    0x31
#define BMX055_ACC_PMU_SELF_TEST 0x32
#define BMX055_ACC_TRIM_NVM_CTRL 0x33
#define BMX055_ACC_BGW_SPI3_WDT  0x34
//#define BMX055_ACC_Reserved    0x35
#define BMX055_ACC_OFC_CTRL      0x36
#define BMX055_ACC_OFC_SETTING   0x37
#define BMX055_ACC_OFC_OFFSET_X  0x38
#define BMX055_ACC_OFC_OFFSET_Y  0x39
#define BMX055_ACC_OFC_OFFSET_Z  0x3A
#define BMX055_ACC_TRIM_GPO      0x3B
#define BMX055_ACC_TRIM_GP1      0x3C
//#define BMX055_ACC_Reserved    0x3D
#define BMX055_ACC_FIFO_CONFIG_1 0x3E
#define BMX055_ACC_FIFO_DATA     0x3F

// BMX055 Gyroscope Registers
#define BMX055_GYRO_WHOAMI           0x00  // should return 0x0F
//#define BMX055_GYRO_Reserved       0x01
#define BMX055_GYRO_RATE_X_LSB       0x02
#define BMX055_GYRO_RATE_X_MSB       0x03
#define BMX055_GYRO_RATE_Y_LSB       0x04
#define BMX055_GYRO_RATE_Y_MSB       0x05
#define BMX055_GYRO_RATE_Z_LSB       0x06
#define BMX055_GYRO_RATE_Z_MSB       0x07
//#define BMX055_GYRO_Reserved       0x08
#define BMX055_GYRO_INT_STATUS_0  0x09
#define BMX055_GYRO_INT_STATUS_1  0x0A
#define BMX055_GYRO_INT_STATUS_2  0x0B
#define BMX055_GYRO_INT_STATUS_3  0x0C
//#define BMX055_GYRO_Reserved    0x0D
#define BMX055_GYRO_FIFO_STATUS   0x0E
#define BMX055_GYRO_RANGE         0x0F
#define BMX055_GYRO_BW            0x10
#define BMX055_GYRO_LPM1          0x11
#define BMX055_GYRO_LPM2          0x12
#define BMX055_GYRO_RATE_HBW      0x13
#define BMX055_GYRO_BGW_SOFTRESET 0x14
#define BMX055_GYRO_INT_EN_0      0x15
#define BMX055_GYRO_INT_EN_1      0x16
#define BMX055_GYRO_INT_MAP_0     0x17
#define BMX055_GYRO_INT_MAP_1     0x18
#define BMX055_GYRO_INT_MAP_2     0x19
#define BMX055_GYRO_INT_SRC_1     0x1A
#define BMX055_GYRO_INT_SRC_2     0x1B
#define BMX055_GYRO_INT_SRC_3     0x1C
//#define BMX055_GYRO_Reserved    0x1D
#define BMX055_GYRO_FIFO_EN       0x1E
//#define BMX055_GYRO_Reserved    0x1F
//#define BMX055_GYRO_Reserved    0x20
#define BMX055_GYRO_INT_RST_LATCH 0x21
#define BMX055_GYRO_HIGH_TH_X     0x22
#define BMX055_GYRO_HIGH_DUR_X    0x23
#define BMX055_GYRO_HIGH_TH_Y     0x24
#define BMX055_GYRO_HIGH_DUR_Y    0x25
#define BMX055_GYRO_HIGH_TH_Z     0x26
#define BMX055_GYRO_HIGH_DUR_Z    0x27
//#define BMX055_GYRO_Reserved    0x28
//#define BMX055_GYRO_Reserved    0x29
//#define BMX055_GYRO_Reserved    0x2A
#define BMX055_GYRO_SOC           0x31
#define BMX055_GYRO_A_FOC         0x32
#define BMX055_GYRO_TRIM_NVM_CTRL 0x33
#define BMX055_GYRO_BGW_SPI3_WDT  0x34
//#define BMX055_GYRO_Reserved    0x35
#define BMX055_GYRO_OFC1          0x36
#define BMX055_GYRO_OFC2          0x37
#define BMX055_GYRO_OFC3          0x38
#define BMX055_GYRO_OFC4          0x39
#define BMX055_GYRO_TRIM_GP0      0x3A
#define BMX055_GYRO_TRIM_GP1      0x3B
#define BMX055_GYRO_BIST          0x3C
#define BMX055_GYRO_FIFO_CONFIG_0 0x3D
#define BMX055_GYRO_FIFO_CONFIG_1 0x3E

// BMX055 magnetometer registers
#define BMX055_MAG_WHOAMI         0x40  // should return 0x32
#define BMX055_MAG_Reserved       0x41
#define BMX055_MAG_XOUT_LSB       0x42
#define BMX055_MAG_XOUT_MSB       0x43
#define BMX055_MAG_YOUT_LSB       0x44
#define BMX055_MAG_YOUT_MSB       0x45
#define BMX055_MAG_ZOUT_LSB       0x46
#define BMX055_MAG_ZOUT_MSB       0x47
#define BMX055_MAG_ROUT_LSB       0x48
#define BMX055_MAG_ROUT_MSB       0x49
#define BMX055_MAG_INT_STATUS     0x4A
#define BMX055_MAG_PWR_CNTL1      0x4B
#define BMX055_MAG_PWR_CNTL2      0x4C
#define BMX055_MAG_INT_EN_1       0x4D
#define BMX055_MAG_INT_EN_2       0x4E
#define BMX055_MAG_LOW_THS        0x4F
#define BMX055_MAG_HIGH_THS       0x50
#define BMX055_MAG_REP_XY         0x51
#define BMX055_MAG_REP_Z          0x52
/* Trim Extended Registers */
#define BMM050_DIG_X1             0x5D // needed for magnetic field calculation
#define BMM050_DIG_Y1             0x5E
#define BMM050_DIG_Z4_LSB         0x62
#define BMM050_DIG_Z4_MSB         0x63
#define BMM050_DIG_X2             0x64
#define BMM050_DIG_Y2             0x65
#define BMM050_DIG_Z2_LSB         0x68
#define BMM050_DIG_Z2_MSB         0x69
#define BMM050_DIG_Z1_LSB         0x6A
#define BMM050_DIG_Z1_MSB         0x6B
#define BMM050_DIG_XYZ1_LSB       0x6C
#define BMM050_DIG_XYZ1_MSB       0x6D
#define BMM050_DIG_Z3_LSB         0x6E
#define BMM050_DIG_Z3_MSB         0x6F
#define BMM050_DIG_XY2            0x70
#define BMM050_DIG_XY1            0x71

// Using SDO1 = SDO2 = CSB3 = GND as designed
// Seven-bit device addresses are ACC = 0x18, GYRO = 0x68, MAG = 0x10
#define BMX055_ACC_ADDRESS  0x18 << 1   // Address of BMX055 accelerometer
#define BMX055_GYRO_ADDRESS 0x68 << 1   // Address of BMX055 gyroscope
#define BMX055_MAG_ADDRESS  0x10 << 1   // Address of BMX055 magnetometer

// Set initial input parameters
// define BMX055 ACC full scale options
#define AFS_2G  0x03
#define AFS_4G  0x05
#define AFS_8G  0x08
#define AFS_16G 0x0C

enum ACCBW {    // define BMX055 accelerometer bandwidths
  ABW_8Hz,      // 7.81 Hz,  64 ms update time
  ABW_16Hz,     // 15.63 Hz, 32 ms update time
  ABW_31Hz,     // 31.25 Hz, 16 ms update time
  ABW_63Hz,     // 62.5  Hz,  8 ms update time
  ABW_125Hz,    // 125   Hz,  4 ms update time
  ABW_250Hz,    // 250   Hz,  2 ms update time
  ABW_500Hz,    // 500   Hz,  1 ms update time
  ABW_100Hz     // 1000  Hz,  0.5 ms update time
};

enum Gscale {
  GFS_2000DPS = 0,
  GFS_1000DPS,
  GFS_500DPS,
  GFS_250DPS,
  GFS_125DPS
};

enum GODRBW {
  G_2000Hz523Hz = 0, // 2000 Hz ODR and unfiltered (bandwidth 523Hz)
  G_2000Hz230Hz,
  G_1000Hz116Hz,
  G_400Hz47Hz,
  G_200Hz23Hz,
  G_100Hz12Hz,
  G_200Hz64Hz,
  G_100Hz32Hz  // 100 Hz ODR and 32 Hz bandwidth
};

enum MODR {
  MODR_10Hz = 0,   // 10 Hz ODR
  MODR_2Hz     ,   // 2 Hz ODR
  MODR_6Hz     ,   // 6 Hz ODR
  MODR_8Hz     ,   // 8 Hz ODR
  MODR_15Hz    ,   // 15 Hz ODR
  MODR_20Hz    ,   // 20 Hz ODR
  MODR_25Hz    ,   // 25 Hz ODR
  MODR_30Hz        // 30 Hz ODR
};

enum Mmode {
  lowPower         = 0,   // rms noise ~1.0 microTesla, 0.17 mA power
  Regular             ,   // rms noise ~0.6 microTesla, 0.5 mA power
  enhancedRegular     ,   // rms noise ~0.5 microTesla, 0.8 mA power
  highAccuracy            // rms noise ~0.3 microTesla, 4.9 mA power
};

// Set up I2C, (SDA,SCL)
I2C i2c2(PB_11, PB_10);

// Specify sensor full scale
uint8_t Ascale = AFS_2G;
uint8_t Gscale = GFS_125DPS;
float aRes, gRes, mRes;     // scale resolutions per LSB for the sensors

// Parameters to hold BMX055 trim values
int8_t      dig_x1;
int8_t      dig_y1;
int8_t      dig_x2;
int8_t      dig_y2;
uint16_t    dig_z1;
int16_t     dig_z2;
int16_t     dig_z3;
int16_t     dig_z4;
uint8_t     dig_xy1;
int8_t      dig_xy2;
uint16_t    dig_xyz1;

// BMX055 variables
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 13/15-bit signed magnetometer sensor output
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0};  // Bias corrections for gyro, accelerometer, mag
float SelfTest[6];            // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = M_PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = M_PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

// Declination at Sao Paulo, Brazil is -21 degrees 7 minutes on 2016-03-27
#define LOCAL_DECLINATION -21.1f

float deltat = 0.0f;          // integration interval for both filter schemes

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method


class BMX055 {
private:
    float pitch, yaw, roll;

    void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
    {
       char data_write[2];
       data_write[0] = subAddress;
       data_write[1] = data;
       i2c2.write(address, data_write, 2, 0);
    }

    char readByte(uint8_t address, uint8_t subAddress)
    {
        char data[1]; // `data` will store the register data
        char data_write[1];
        data_write[0] = subAddress;
        i2c2.write(address, data_write, 1, 1); // no stop
        i2c2.read(address, data, 1, 0);
        return data[0];
    }

    void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
    {
        char data[14];
        char data_write[1];
        data_write[0] = subAddress;
        i2c2.write(address, data_write, 1, 1); // no stop
        i2c2.read(address, data, count, 0);
        for(int ii = 0; ii < count; ii++)
        {
            dest[ii] = data[ii];
        }
    }

public:
    BMX055()
    {
        //Set up I2C
        i2c2.frequency(400000);  // use fast (400 kHz) I2C
    }

    float getAres(void)
    {
        switch (Ascale)
        {
            // Possible accelerometer scales (and their register bit settings) are:
            // 2 Gs (0011), 4 Gs (0101), 8 Gs (1000), and 16 Gs  (1100).
            // BMX055 ACC data is signed 12 bit
            case AFS_2G:
                aRes = 2.0/2048.0;
                break;
            case AFS_4G:
                aRes = 4.0/2048.0;
                break;
            case AFS_8G:
                aRes = 8.0/2048.0;
                break;
            case AFS_16G:
                aRes = 16.0/2048.0;
                break;
        }

        return aRes;
    }

    float getGres(void)
    {
        switch (Gscale)
        {
            // Possible gyro scales (and their register bit settings) are:
            // 125 DPS (100), 250 DPS (011), 500 DPS (010), 1000 DPS (001), and 2000 DPS (000).
            case GFS_125DPS:
                gRes = 124.87/32768.0; // per data sheet, not exactly 125!?
                break;
            case GFS_250DPS:
                gRes = 249.75/32768.0;
                break;
            case GFS_500DPS:
                gRes = 499.5/32768.0;
                break;
            case GFS_1000DPS:
                gRes = 999.0/32768.0;
                break;
            case GFS_2000DPS:
                gRes = 1998.0/32768.0;
                break;
        }

        return gRes;
    }

    float getPitch(void)
    {
        return pitch;
    }

    float getRoll(void)
    {
        return roll;
    }

    float getYaw(void)
    {
        return yaw;
    }

    void readAccelData(int16_t * destination)
    {
        uint8_t rawData[6];  // x/y/z accel register data stored here

        // Read the six raw data registers into data array
        readBytes(BMX055_ACC_ADDRESS, BMX055_ACC_D_X_LSB, 6, &rawData[0]);
        if((rawData[0] & 0x01) && (rawData[2] & 0x01) && (rawData[4] & 0x01))
        {  // Check that all 3 axes have new data
            // Turn the MSB and LSB into a signed 12-bit value
            destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]) >> 4;
            destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]) >> 4;
            destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]) >> 4;
        }
    }

    void readGyroData(int16_t * destination)
    {
        uint8_t rawData[6];  // x/y/z gyro register data stored here

        // Read the six raw data registers sequentially into data array
        readBytes(BMX055_GYRO_ADDRESS, BMX055_GYRO_RATE_X_LSB, 6, &rawData[0]);
        // Turn the MSB and LSB into a signed 16-bit value
        destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);
        destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);
        destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]);
    }

    void readMagData(int16_t * magData)
    {
        int16_t mdata_x = 0, mdata_y = 0, mdata_z = 0, temp = 0;
        uint16_t data_r = 0;
        uint8_t rawData[8];  // x/y/z hall magnetic field data, and Hall resistance data
        readBytes(BMX055_MAG_ADDRESS, BMX055_MAG_XOUT_LSB, 8, &rawData[0]);  // Read the eight raw data registers sequentially into data array
        if(rawData[6] & 0x01) { // Check if data ready status bit is set
            mdata_x = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]) >> 3;  // 13-bit signed integer for x-axis field
            mdata_y = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]) >> 3;  // 13-bit signed integer for y-axis field
            mdata_z = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]) >> 1;  // 15-bit signed integer for z-axis field
            data_r = (uint16_t) (((uint16_t)rawData[7] << 8) | rawData[6]) >> 2;  // 14-bit unsigned integer for Hall resistance

            // calculate temperature compensated 16-bit magnetic fields
            temp = ((int16_t)(((uint16_t)((((int32_t)dig_xyz1) << 14)/(data_r != 0 ? data_r : dig_xyz1))) - ((uint16_t)0x4000)));
            magData[0] = ((int16_t)((((int32_t)mdata_x) *
            ((((((((int32_t)dig_xy2) * ((((int32_t)temp) * ((int32_t)temp)) >> 7)) +
            (((int32_t)temp) * ((int32_t)(((int16_t)dig_xy1) << 7)))) >> 9) +
            ((int32_t)0x100000)) * ((int32_t)(((int16_t)dig_x2) + ((int16_t)0xA0)))) >> 12)) >> 13)) +
            (((int16_t)dig_x1) << 3);

            temp = ((int16_t)(((uint16_t)((((int32_t)dig_xyz1) << 14)/(data_r != 0 ? data_r : dig_xyz1))) - ((uint16_t)0x4000)));
            magData[1] = ((int16_t)((((int32_t)mdata_y) *
            ((((((((int32_t)dig_xy2) * ((((int32_t)temp) * ((int32_t)temp)) >> 7)) +
            (((int32_t)temp) * ((int32_t)(((int16_t)dig_xy1) << 7)))) >> 9) +
            ((int32_t)0x100000)) * ((int32_t)(((int16_t)dig_y2) + ((int16_t)0xA0)))) >> 12)) >> 13)) +
            (((int16_t)dig_y1) << 3);
            magData[2] = (((((int32_t)(mdata_z - dig_z4)) << 15) - ((((int32_t)dig_z3) * ((int32_t)(((int16_t)data_r) -
            ((int16_t)dig_xyz1))))>>2))/(dig_z2 + ((int16_t)(((((int32_t)dig_z1) * ((((int16_t)data_r) << 1)))+(1<<15))>>16))));
        }
    }

    float getTemperature()
    {
        uint8_t c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_D_TEMP);  // Read the raw data register
        int16_t tempCount = ((int16_t)((int16_t)c << 8)) >> 8 ;  // Turn the byte into a signed 8-bit integer

        return ((((float)tempCount) * 0.5) + 23.0);   // temperature in degrees Centigrade
    }

    void fastcompaccelBMX055(float * dest1)
    {
        writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x80); // set all accel offset compensation registers to zero
        writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_SETTING, 0x20);  // set offset targets to 0, 0, and +1 g for x, y, z axes
        writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x20); // calculate x-axis offset

        uint8_t c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
        while(!(c & 0x10))
        {   // check if fast calibration complete
            c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
            wait_ms(10);
        }
        writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x40); // calculate y-axis offset

        c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
        while(!(c & 0x10))
        {   // check if fast calibration complete
            c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
            wait_ms(10);
        }
        writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x60); // calculate z-axis offset

        c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
        while(!(c & 0x10))
        {   // check if fast calibration complete
            c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
            wait_ms(10);
        }

        int8_t compx = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_X);
        int8_t compy = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_Y);
        int8_t compz = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_Z);

        dest1[0] = (float) compx/128.; // accleration bias in g
        dest1[1] = (float) compy/128.; // accleration bias in g
        dest1[2] = (float) compz/128.; // accleration bias in g
    }

    void magcalBMX055(float * dest1)
    {
        uint16_t ii = 0, sample_count = 0;
        int32_t mag_bias[3] = {0, 0, 0};
        int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};

        // ## "Mag Calibration: Wave device in a figure eight until done!" ##
        //wait(4);

        sample_count = 48;
        for(ii = 0; ii < sample_count; ii++)
        {
            int16_t mag_temp[3] = {0, 0, 0};
            readMagData(mag_temp);
            for (int jj = 0; jj < 3; jj++)
            {
                if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
                if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
            }
            wait_ms(105);  // at 10 Hz ODR, new mag data is available every 100 ms
        }

        mag_bias[0] = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
        mag_bias[1] = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
        mag_bias[2] = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

        // save mag biases in G for main program
        dest1[0] = (float) mag_bias[0]*mRes;
        dest1[1] = (float) mag_bias[1]*mRes;
        dest1[2] = (float) mag_bias[2]*mRes;

        // ## "Mag Calibration done!" ##
    }

    // Get trim values for magnetometer sensitivity
    void trim(void)
    {
        uint8_t rawData[2];  //placeholder for 2-byte trim data

        dig_x1 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_X1);
        dig_x2 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_X2);
        dig_y1 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_Y1);
        dig_y2 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_Y2);
        dig_xy1 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_XY1);
        dig_xy2 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_XY2);
        readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z1_LSB, 2, &rawData[0]);
        dig_z1 = (uint16_t) (((uint16_t)rawData[1] << 8) | rawData[0]);
        readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z2_LSB, 2, &rawData[0]);
        dig_z2 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);
        readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z3_LSB, 2, &rawData[0]);
        dig_z3 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);
        readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z4_LSB, 2, &rawData[0]);
        dig_z4 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);
        readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_XYZ1_LSB, 2, &rawData[0]);
        dig_xyz1 = (uint16_t) (((uint16_t)rawData[1] << 8) | rawData[0]);
    }

    /** Initialize device for active mode
     * @param Ascale set accel full scale
     * @param ACCBW set bandwidth for accelerometer
     * @param Gscale set gyro full scale
     * @param GODRBW set gyro ODR and bandwidth
     * @param Mmode set magnetometer operation mode
     * @param MODR set magnetometer data rate
     * @see ACCBW, GODRBW and MODR enums
     */
    void init(uint8_t mAscale = AFS_2G, uint8_t ACCBW  = ABW_16Hz,
                uint8_t mGscale = GFS_125DPS, uint8_t GODRBW = G_200Hz23Hz,
                uint8_t Mmode  = Regular, uint8_t MODR   = MODR_10Hz)
    {
        Ascale = mAscale;
        Gscale = mGscale;

        // Configure accelerometer
        writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_PMU_RANGE, Ascale & 0x0F); // Set accelerometer full scale
        writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_PMU_BW, (0x08 | ACCBW) & 0x0F);     // Set accelerometer bandwidth
        writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_D_HBW, 0x00);              // Use filtered data

        // Configure Gyro
        writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_RANGE, Gscale);  // set GYRO FS range
        writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_BW, GODRBW);     // set GYRO ODR and Bandwidth

        // Configure magnetometer
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x82);  // Softreset magnetometer, ends up in sleep mode
        wait_ms(100);
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x01); // Wake up magnetometer
        wait_ms(100);
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL2, MODR << 3); // Normal mode

        // Set up four standard configurations for the magnetometer
        switch (Mmode)
        {
            case lowPower:
                // Low-power
                writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x01);  // 3 repetitions (oversampling)
                writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x02);  // 3 repetitions (oversampling)
                break;
            case Regular:
                // Regular
                writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x04);  //  9 repetitions (oversampling)
                writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x16);  // 15 repetitions (oversampling)
                break;
            case enhancedRegular:
                // Enhanced Regular
                writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x07);  // 15 repetitions (oversampling)
                writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x22);  // 27 repetitions (oversampling)
                break;
            case highAccuracy:
                // High Accuracy
                writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x17);  // 47 repetitions (oversampling)
                writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x51);  // 83 repetitions (oversampling)
                break;
        }

        // get sensor resolutions, only need to do this once
        getAres();
        getGres();
        // magnetometer resolution is 1 microTesla/16 counts or 1/1.6 milliGauss/count
        mRes = 1./1.6;

        trim(); // read the magnetometer calibration data

        fastcompaccelBMX055(accelBias);
        magcalBMX055(magBias);
        // TODO: see magcalBMX055(): 128 samples * 105ms = 13.44s
        // So far, magnetometer bias is calculated and subtracted here manually, should construct an algorithm to do it automatically
        // like the gyro and accelerometer biases
        // magBias[0] = -5.;   // User environmental x-axis correction in milliGauss
        // magBias[1] = -95.;  // User environmental y-axis correction in milliGauss
        // magBias[2] = -260.; // User environmental z-axis correction in milliGauss

    }

    // Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
    // (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
    // which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
    // device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
    // The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
    // but is much less computationally intensive
    void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
    {
        float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
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
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f/norm;
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalise magnetometer measurement
        norm = sqrt(mx * mx + my * my + mz * mz);
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f/norm;
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
        norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
        norm = 1.0f/norm;
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
        s4 *= norm;

        // Compute rate of change of quaternion
        qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
        qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
        qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
        qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

        // Integrate to yield quaternion
        q1 += qDot1 * deltat;
        q2 += qDot2 * deltat;
        q3 += qDot3 * deltat;
        q4 += qDot4 * deltat;
        norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
        norm = 1.0f/norm;
        q[0] = q1 * norm;
        q[1] = q2 * norm;
        q[2] = q3 * norm;
        q[3] = q4 * norm;

    }

    /** Get raw 9-axis motion sensor readings (accel/gyro/compass).
     * @param ax 12-bit signed integer container for accelerometer X-axis value
     * @param ay 12-bit signed integer container for accelerometer Y-axis value
     * @param az 12-bit signed integer container for accelerometer Z-axis value
     * @param gx 16-bit signed integer container for gyroscope X-axis value
     * @param gy 16-bit signed integer container for gyroscope Y-axis value
     * @param gz 16-bit signed integer container for gyroscope Z-axis value
     * @param mx 13-bit signed integer container for magnetometer X-axis value
     * @param my 13-bit signed integer container for magnetometer Y-axis value
     * @param mz 15-bit signed integer container for magnetometer Z-axis value
     * @see getAcceleration()
     * @see getRotation()
     * @see getMag()
     */
    void getRaw9(	int16_t* ax, int16_t* ay, int16_t* az,
    					int16_t* gx, int16_t* gy, int16_t* gz,
    					int16_t* mx, int16_t* my, int16_t* mz)
    {
    	uint8_t rawData[8];  // x/y/z MSB and LSB registers raw data stored here

        // Read the six raw data registers into data array
        // Turn the MSB and LSB into a signed 12-bit value
    	readBytes(BMX055_ACC_ADDRESS, BMX055_ACC_D_X_LSB, 6, rawData);
    	*ax = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]) >> 4;
    	*ay = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) >> 4;
    	*az = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) >> 4;

        // Read the six raw data registers sequentially into data array
        // Turn the MSB and LSB into a signed 16-bit value
    	readBytes(BMX055_GYRO_ADDRESS, BMX055_GYRO_RATE_X_LSB, 6, rawData);
    	*gx = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);
    	*gy = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]);
    	*gz = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]);

        // Read the six raw data registers into data array
        // 13-bit signed integer for x-axis and y-axis field
        // 15-bit signed integer for z-axis field
    	readBytes(BMX055_MAG_ADDRESS, BMX055_MAG_XOUT_LSB, 8, rawData);
        if(rawData[6] & 0x01)   // Check if data ready status bit is set
        {
            *mx = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]) >> 3;
            *my = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) >> 3;
            *mz = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) >> 1;
        }
    }

    /** Get raw 9-axis motion sensor readings (accel/gyro/compass).
     * @param ax accelerometer X-axis value (g's)
     * @param ay accelerometer Y-axis value (g's)
     * @param az accelerometer Z-axis value (g's)
     * @param gx gyroscope X-axis value (degrees per second)
     * @param gy gyroscope Y-axis value (degrees per second)
     * @param gz gyroscope Z-axis value (degrees per second)
     * @param mx magnetometer X-axis value (milliGauss)
     * @param my magnetometer Y-axis value (milliGauss)
     * @param mz magnetometer Z-axis value (milliGauss)
     * @see getAcceleration()
     * @see getRotation()
     * @see getMag()
     */
    void getMotion9(	float* ax, float* ay, float* az,
                        float* gx, float* gy, float* gz,
                        float* mx, float* my, float* mz)
    {
        int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
        int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
        int16_t magCount[3];    // Stores the 13/15-bit signed magnetometer sensor output

        // Read the x/y/z raw values
        readAccelData(accelCount);

        // Calculate the accleration value into actual g's
        // get actual g value, this depends on scale being set
        *ax = (float)accelCount[0]*aRes; // + accelBias[0];
        *ay = (float)accelCount[1]*aRes; // + accelBias[1];
        *az = (float)accelCount[2]*aRes; // + accelBias[2];


        // Read the x/y/z raw values
        readGyroData(gyroCount);

        // Calculate the gyro value into actual degrees per second
        // get actual gyro value, this depends on scale being set
        *gx = (float)gyroCount[0]*gRes;
        *gy = (float)gyroCount[1]*gRes;
        *gz = (float)gyroCount[2]*gRes;


        // Read the x/y/z raw values
        readMagData(magCount);

        // Calculate the magnetometer values in milliGauss
        // Temperature-compensated magnetic field is in 16 LSB/microTesla
        // get actual magnetometer value, this depends on scale being set
        *mx = (float)magCount[0]*mRes - magBias[0];
        *my = (float)magCount[1]*mRes - magBias[1];
        *mz = (float)magCount[2]*mRes - magBias[2];
    }

    void runAHRS(float mdeltat, float local_declination = LOCAL_DECLINATION)
    {
        float ax, ay, az, gx, gy, gz, mx, my, mz;

        getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        deltat = mdeltat;

        // Sensors x (y)-axis of the accelerometer is aligned with the -y (x)-axis of the magnetometer;
        // the magnetometer z-axis (+ up) is aligned with z-axis (+ up) of accelerometer and gyro!
        // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
        // For the BMX-055, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
        // in the MPU9250 sensor. This rotation can be modified to allow any convenient orientation convention.
        // This is ok by aircraft orientation standards!
        // Pass gyro rate as rad/s
        //MadgwickQuaternionUpdate(ax, ay, az, gx*M_PI/180.0f, gy*M_PI/180.0f, gz*M_PI/180.0f, -my, mx, mz);
        MadgwickQuaternionUpdate(-ay, ax, az, -gy*M_PI/180.0f, gx*M_PI/180.0f, gz*M_PI/180.0f, mx, my, mz);


        // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
        // In this coordinate system, the positive z-axis is down toward Earth.
        // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
        // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
        // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
        // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
        // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
        // applied in the correct order which for this configuration is yaw, pitch, and then roll.
        // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
        yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
        pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
        roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
        pitch *= 180.0f / M_PI;
        yaw   *= 180.0f / M_PI;
        yaw   -= local_declination;
        roll  *= 180.0f / M_PI;
    }
};

#endif /* BMX055_H_ */
