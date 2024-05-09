#pragma once

#include "vector3.h"
#include "matrix3.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
// #include <TP_Sense/Adafruit_LSM9DS1_Library/Adafruit_LSM9DS1.h>
// #include <TP_Sense/Adafruit_Unified_Sensor/Adafruit_Sensor.h>  // not used in this demo but required!

#define LSM9DS1_ADDRESS_ACCELGYRO (0x6B)
#define LSM9DS1_ADDRESS_MAG (0x1E)
#define LSM9DS1_XG_ID (0b01101000)


// Linear Acceleration: mg per LSB
#define LSM9DS1_ACCEL_MG_LSB_2G (0.061F)
#define LSM9DS1_ACCEL_MG_LSB_4G (0.122F)
#define LSM9DS1_ACCEL_MG_LSB_8G (0.244F) // used
#define LSM9DS1_ACCEL_MG_LSB_16G (0.732F)

// Magnetic Field Strength: gauss range
#define LSM9DS1_MAG_MGAUSS_4GAUSS (0.14F)
#define LSM9DS1_MAG_MGAUSS_8GAUSS (0.29F) //used
#define LSM9DS1_MAG_MGAUSS_12GAUSS (0.43F)
#define LSM9DS1_MAG_MGAUSS_16GAUSS (0.58F)

// Angular Rate: dps per LSB
#define LSM9DS1_GYRO_DPS_DIGIT_245DPS (0.00875F)
#define LSM9DS1_GYRO_DPS_DIGIT_500DPS (0.01750F) //used
#define LSM9DS1_GYRO_DPS_DIGIT_2000DPS (0.07000F)

// Magnetometer  Registers
#define LIS3MDL_REG_WHO_AM_I 0x0F  ///< Register that contains the part ID
#define LIS3MDL_REG_CTRL_REG1 0x20 ///< Register address for control 1
#define LIS3MDL_REG_CTRL_REG2 0x21 ///< Register address for control 2
#define LIS3MDL_REG_CTRL_REG3 0x22 ///< Register address for control 3
#define LIS3MDL_REG_CTRL_REG4 0x23 ///< Register address for control 3
#define LIS3MDL_REG_STATUS 0x27    ///< Register address for status
#define LIS3MDL_REG_OUT_X_L 0x28   ///< Register address for X axis lower byte
#define LIS3MDL_REG_INT_CFG 0x30   ///< Interrupt configuration register
#define LIS3MDL_REG_INT_THS_L 0x32 ///< Low byte of the irq threshold

/** The magnetometer ranges */
typedef enum {
  LIS3MDL_RANGE_4_GAUSS = 0b00,  ///< +/- 4g (default value)
  LIS3MDL_RANGE_8_GAUSS = 0b01,  ///< +/- 8g
  LIS3MDL_RANGE_12_GAUSS = 0b10, ///< +/- 12g
  LIS3MDL_RANGE_16_GAUSS = 0b11, ///< +/- 16g
} lis3mdl_range_t;

/** The magnetometer data rate, includes FAST_ODR bit */
typedef enum {
  LIS3MDL_DATARATE_0_625_HZ = 0b0000, ///<  0.625 Hz
  LIS3MDL_DATARATE_1_25_HZ = 0b0010,  ///<  1.25 Hz
  LIS3MDL_DATARATE_2_5_HZ = 0b0100,   ///<  2.5 Hz
  LIS3MDL_DATARATE_5_HZ = 0b0110,     ///<  5 Hz
  LIS3MDL_DATARATE_10_HZ = 0b1000,    ///<  10 Hz
  LIS3MDL_DATARATE_20_HZ = 0b1010,    ///<  20 Hz
  LIS3MDL_DATARATE_40_HZ = 0b1100,    ///<  40 Hz
  LIS3MDL_DATARATE_80_HZ = 0b1110,    ///<  80 Hz
  LIS3MDL_DATARATE_155_HZ = 0b0001,   ///<  155 Hz (FAST_ODR + UHP)
  LIS3MDL_DATARATE_300_HZ = 0b0011,   ///<  300 Hz (FAST_ODR + HP)
  LIS3MDL_DATARATE_560_HZ = 0b0101,   ///<  560 Hz (FAST_ODR + MP)
  LIS3MDL_DATARATE_1000_HZ = 0b0111,  ///<  1000 Hz (FAST_ODR + LP)
} lis3mdl_dataRate_t;

/** The magnetometer performance mode */
typedef enum {
  LIS3MDL_LOWPOWERMODE = 0b00,  ///< Low power mode
  LIS3MDL_MEDIUMMODE = 0b01,    ///< Medium performance mode
  LIS3MDL_HIGHMODE = 0b10,      ///< High performance mode
  LIS3MDL_ULTRAHIGHMODE = 0b11, ///< Ultra-high performance mode
} lis3mdl_performancemode_t;

/** The magnetometer operation mode */
typedef enum {
  LIS3MDL_CONTINUOUSMODE = 0b00, ///< Continuous conversion
  LIS3MDL_SINGLEMODE = 0b01,     ///< Single-shot conversion
  LIS3MDL_POWERDOWNMODE = 0b11,  ///< Powered-down mode
} lis3mdl_operationmode_t;

class LSM9DS1
{
// private:

    //     Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
    /**! Register mapping for accelerometer/gyroscope component */
    typedef enum {
        LSM9DS1_REGISTER_WHO_AM_I_XG = 0x0F,
        LSM9DS1_REGISTER_CTRL_REG1_G = 0x10,
        LSM9DS1_REGISTER_CTRL_REG2_G = 0x11,
        LSM9DS1_REGISTER_CTRL_REG3_G = 0x12,
        LSM9DS1_REGISTER_TEMP_OUT_L = 0x15,
        LSM9DS1_REGISTER_TEMP_OUT_H = 0x16,
        LSM9DS1_REGISTER_STATUS_REG = 0x17,
        LSM9DS1_REGISTER_OUT_X_L_G = 0x18,
        LSM9DS1_REGISTER_OUT_X_H_G = 0x19,
        LSM9DS1_REGISTER_OUT_Y_L_G = 0x1A,
        LSM9DS1_REGISTER_OUT_Y_H_G = 0x1B,
        LSM9DS1_REGISTER_OUT_Z_L_G = 0x1C,
        LSM9DS1_REGISTER_OUT_Z_H_G = 0x1D,
        LSM9DS1_REGISTER_CTRL_REG4 = 0x1E,
        LSM9DS1_REGISTER_CTRL_REG5_XL = 0x1F,
        LSM9DS1_REGISTER_CTRL_REG6_XL = 0x20,
        LSM9DS1_REGISTER_CTRL_REG7_XL = 0x21,
        LSM9DS1_REGISTER_CTRL_REG8 = 0x22,
        LSM9DS1_REGISTER_CTRL_REG9 = 0x23,
        LSM9DS1_REGISTER_CTRL_REG10 = 0x24,

        LSM9DS1_REGISTER_OUT_X_L_XL = 0x28,
        LSM9DS1_REGISTER_OUT_X_H_XL = 0x29,
        LSM9DS1_REGISTER_OUT_Y_L_XL = 0x2A,
        LSM9DS1_REGISTER_OUT_Y_H_XL = 0x2B,
        LSM9DS1_REGISTER_OUT_Z_L_XL = 0x2C,
        LSM9DS1_REGISTER_OUT_Z_H_XL = 0x2D,
    } lsm9ds1AccGyroRegisters_t;

    /**! Enumeration for accelerometer range (2/4/8/16 g) */
    typedef enum {
        LSM9DS1_ACCELRANGE_2G = (0b00 << 3),
        LSM9DS1_ACCELRANGE_16G = (0b01 << 3),
        LSM9DS1_ACCELRANGE_4G = (0b10 << 3),
        LSM9DS1_ACCELRANGE_8G = (0b11 << 3),
    } lsm9ds1AccelRange_t;

    /**! Enumeration for accelerometer data rate 10 - 952 Hz */
    typedef enum {
        LSM9DS1_ACCELDATARATE_POWERDOWN = (0b0000 << 5),
        LSM9DS1_ACCELDATARATE_10HZ = (0b001 << 5),
        LSM9DS1_ACCELDATARATE_50HZ = (0b010 << 5),
        LSM9DS1_ACCELDATARATE_119HZ = (0b011 << 5),
        LSM9DS1_ACCELDATARATE_238HZ = (0b100 << 5),
        LSM9DS1_ACCELDATARATE_476HZ = (0b101 << 5),
        LSM9DS1_ACCELDATARATE_952HZ = (0b110 << 5),
    } lsm9ds1AccelDataRate_t;

    /**! Enumeration for magnetometer scaling (4/8/12/16 gauss) */
    typedef enum {
        LSM9DS1_MAGGAIN_4GAUSS = (0b00 << 5),  // +/- 4 gauss
        LSM9DS1_MAGGAIN_8GAUSS = (0b01 << 5),  // +/- 8 gauss
        LSM9DS1_MAGGAIN_12GAUSS = (0b10 << 5), // +/- 12 gauss
        LSM9DS1_MAGGAIN_16GAUSS = (0b11 << 5)  // +/- 16 gauss
    } lsm9ds1MagGain_t;

    /**! Enumeration for gyroscope scaling (245/500/2000 dps) */
    typedef enum {
        LSM9DS1_GYROSCALE_245DPS =
            (0b00 << 3), // +/- 245 degrees per second rotation
        LSM9DS1_GYROSCALE_500DPS =
            (0b01 << 3), // +/- 500 degrees per second rotation
        LSM9DS1_GYROSCALE_2000DPS =
            (0b11 << 3) // +/- 2000 degrees per second rotation
    } lsm9ds1GyroScale_t;

    Vector3f AccelOffset;

    Vector3f HardOffsetVect;
    Matrix3f SoftCalibMat;
    Rotation TO_NED_FRAME = ROTATION_ROLL_180_YAW_90;

protected:
public:
    const float std_dev_mag = 0.8;
    Vector3f MagRaw;
    Vector3f MagVect;
    Vector3f UnitMagVect;
    Vector3f m_ref;
    Vector3f GyroOffset;
    Vector3f a_ref;

    // const float std_dev_gyro = 4*DEG_TO_RAD;     // 4deg/sec (try 1) 0.0698132rad/sec
    // const float std_dev_accel = 3*DEG_TO_RAD;    // 3deg try 1 0.0523599??
    Vector3f GyroRate;
    Vector3f AccelBody;       // Measurements from Accel
    Vector3f UnitAccVect;

    float accel_mg_lsb, gyro_dps_digit;
    // Rotation TO_NED_FRAME = ROTATION_ROLL_180;
    LSM9DS1(){}
    void write8(byte addr, byte reg, byte value) {
        Wire.beginTransmission(addr);
        Wire.write(reg);  // Register 107 - Power Management 1
        Wire.write(value);  // To start the device
        Wire.endTransmission();
    }
    // byte read8(byte addr, byte reg){
    //     uint8_t value;
    //     Wire.beginTransmission(addr);
    //     Wire.write(0x43);                                                //Register 67-72 - Gyroscope Measurements
    //     Wire.endTransmission();
    //     Wire.requestFrom(addr, 1);
    //     value = Wire.read();
    //     return value;
    // }
    bool sensors_setup() {
        // I2C
        // i2c_dev = new Adafruit_I2CDevice(LSM9DS1_ADDRESS_ACCELGYRO, _wire);
        // if (!i2c_dev->begin())
        // return false;
        // if (!_magSensor.begin_I2C(LSM9DS1_ADDRESS_MAG, _wire))
        // return false;

        // soft reset & reboot accel/gyro
        write8(LSM9DS1_ADDRESS_ACCELGYRO, LSM9DS1_REGISTER_CTRL_REG8, 0x05);

        delay(10);

        // uint8_t id = read8(LSM9DS1_ADDRESS_ACCELGYRO, LSM9DS1_REGISTER_WHO_AM_I_XG);
        // if (id != LSM9DS1_XG_ID)
        //     return false;

        uint8_t val = 0x00;
        // enable gyro continuous; Output Data Rate 952Hz, Cuttoff 100Hz (after LPF)
        val = 0xC0;
        val |= LSM9DS1_GYROSCALE_500DPS;
        // no need for REG4 enables? 
        write8(LSM9DS1_ADDRESS_ACCELGYRO, LSM9DS1_REGISTER_CTRL_REG1_G, val); // on XYZ
        gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_500DPS;

        // Enable the accelerometer continous and set range
        write8(LSM9DS1_ADDRESS_ACCELGYRO, LSM9DS1_REGISTER_CTRL_REG5_XL, 0x38); // enable X Y and Z axis
        write8(LSM9DS1_ADDRESS_ACCELGYRO, LSM9DS1_REGISTER_CTRL_REG6_XL, 0xC0); 
        val = 0x00; // bottom 3 bits are set to 0 pg52/72
        val |= LSM9DS1_ACCELRANGE_8G | LSM9DS1_ACCELDATARATE_952HZ;
        write8(LSM9DS1_ADDRESS_ACCELGYRO, LSM9DS1_REGISTER_CTRL_REG6_XL, val);
        accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_2G;

        // enable mag continuous
        write8(LSM9DS1_ADDRESS_MAG, LIS3MDL_REG_CTRL_REG3, LIS3MDL_CONTINUOUSMODE);
        val = 0x00;
        val |= LSM9DS1_MAGGAIN_4GAUSS;
        write8(LSM9DS1_ADDRESS_MAG, LIS3MDL_REG_CTRL_REG2, val);
        
        // HardOffsetVect.set(-720.637, 1365.0, 2270.015);
        HardOffsetVect.set(-587.808893, 1041.364679, 2152.803319);

        // float temp[6] =  {1.29, 1.31, 1.28, 0.027, -0.0698, 0.01394};
        // float temp[6] =  {0.670999, 0.688844, 0.518978, 0.008017, 0.080208, -0.002301};
        float temp[6] =  {0.547561, 0.553483, 0.590261, -0.023879, -0.005508, -0.002054};
        SoftCalibMat.a.set(temp[0], temp[3], temp[4]);
        SoftCalibMat.b.set(temp[3], temp[1], temp[5]);
        SoftCalibMat.c.set(temp[4], temp[5], temp[2]);

        return true;
    }
    void read_sensors() { 
        Wire.beginTransmission(LSM9DS1_ADDRESS_ACCELGYRO);
        Wire.write(LSM9DS1_REGISTER_OUT_X_L_G);                                                //Register 67-72 - Gyroscope Measurements
        Wire.endTransmission();
        Wire.requestFrom(LSM9DS1_ADDRESS_ACCELGYRO, 6);
        Vector3i Gyro; // all neg
        Gyro.x =  (Wire.read() | Wire.read() << 8);                  //67 GYRO_XOUT[15:8] and 68 GYRO_XOUT[7:0]
        Gyro.y =  (Wire.read() | Wire.read() << 8);                  //67 GYRO_XOUT[15:8] and 68 GYRO_XOUT[7:0]
        Gyro.z = -(Wire.read() | Wire.read() << 8);                  //67 GYRO_XOUT[15:8] and 68 GYRO_XOUT[7:0]
        GyroRate = Gyro.tofloat() / 65.536 *DEG_TO_RAD;  //integer to deg/sec  /////////////////////////////////work with int instead?
        // GyroRate.rotate(TO_NED_FRAME);
        GyroRate -= GyroOffset;


        Wire.beginTransmission(LSM9DS1_ADDRESS_ACCELGYRO);
        Wire.write(LSM9DS1_REGISTER_OUT_X_L_XL);  //Register 59-64 - Accelerometer Measurements
        Wire.endTransmission();
        Wire.requestFrom(LSM9DS1_ADDRESS_ACCELGYRO, 6);
        Vector3i AccLSB; // all pos
        AccLSB.x =  -(Wire.read() | Wire.read() << 8);  //59 ACCEL_XOUT[15:8] and 60 ACCEL_XOUT[7:0] -
        AccLSB.y =  -(Wire.read() | Wire.read() << 8);  //61 ACCEL_YOUT[15:8] and 62 ACCEL_YOUT[7:0] -
        AccLSB.z =  (Wire.read() | Wire.read() << 8);  //63 ACCEL_ZOUT[15:8] and 64 ACCEL_ZOUT[7:0] +
        // to fix sign to MPU6050 frame negative stored

        AccelBody = AccLSB.tofloat()/4096 + AccelOffset;
        // AccelBody.rotate(TO_NED_FRAME);
        UnitAccVect = AccelBody.normalized();

        
        Wire.beginTransmission(LSM9DS1_ADDRESS_MAG);
        Wire.write(LIS3MDL_REG_OUT_X_L);
        int err = Wire.endTransmission();
        if(!err){
            Wire.requestFrom(LSM9DS1_ADDRESS_MAG, 6);
            MagRaw.x = -(float)(int16_t)(Wire.read()|Wire.read()<<8);    // -
            MagRaw.y = (float)(int16_t)(Wire.read()|Wire.read()<<8);    // +
            MagRaw.z = -(float)(int16_t)(Wire.read()|Wire.read()<<8);    // -
        }
        //Calibration
        MagVect = SoftCalibMat*(MagRaw-HardOffsetVect);
        // MagRaw.rotate(TO_NED_FRAME);
        UnitMagVect = MagVect.normalized();
//   readTemp();

    }
    
    void calibrate_gyro(){
        GyroOffset.zero();
        Vector3f temp_sum; //since offsets are used in read sensor functions
        for (int i = 0; i < 1000; i++) {
            read_sensors();
            temp_sum += GyroRate;
            delayMicroseconds(100);
        }
        GyroOffset = temp_sum/1000;
    }
    void set_m_ref(){
        Vector3f sum;
        for(int i = 0; i<1000; i++){
            read_sensors();
            sum += UnitMagVect;
        }
        m_ref = sum/=1000;
        // m_ref.set(38290.3, 0, 20923.0); //NED
        m_ref.normalize();
    }    
    
    void set_a_ref(){
        Vector3f sum;
        for(int i = 0; i<1000; i++){
            read_sensors();
            sum += UnitAccVect;
        }
        a_ref = sum/=1000;
        // m_ref.set(38290.3, 0, 20923.0); //NED
        a_ref.normalize();
    }
};
