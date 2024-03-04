#pragma once

#include "vector3.h"
#include <Arduino.h>
#include <Wire.h>

#define MPU_ADDR 0x68

class MPU6050
{
private:
    Vector3f AccelOffset;

protected:
public:
    Vector3f GyroOffset;
    Vector3f a_ref;

    const float std_dev_gyro = 0.3;     // 4deg/sec (try 1) 0.0698132rad/sec
    const float std_dev_accel = 0.5;    // 3deg try 1 0.0523599??
    Vector3f GyroRate;
    Vector3f AccelBody;       // Measurements from Accel
    Vector3f UnitAccelBody;
    Rotation TO_NED_FRAME = ROTATION_ROLL_180;
    MPU6050(){}
    void mpu_setup() {  //MPU6050 default address is 0x68 MPU:Motion Processing Units
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x6B);  // Register 107 - Power Management 1
        Wire.write(0x00);  // To start the device
        Wire.endTransmission();

        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x1A);  //Register 26 - Configuration
        Wire.write(0x05);  //DLPF settings for gyro and accel
                            //0x05 Accel and Gyro 1kHz: Bandwidth 10Hz Delay 13.8/13.4(and disables FSYNC?)
                            //0x00 Accel 260Hz bandwidth 0ms delay, No DLPF
        Wire.endTransmission();
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x1B);  //Register 27 - Gyro Config
        Wire.write(0x08);   //Full Scale Range +/- 500deg/sec -> Sensitivity of the Gyro to 65.5 LSB/deg/sec 2^16/(2*500)=65.536
        Wire.endTransmission();
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x1C);   //Register 28 - Accel Config
        Wire.write(0x10);   //Full Scale Range +/- 8g         -> Sensitivity of the Accel to 4096 LSB/g = 2^16/(2*8)
                            // 1g = 4096LSB
        Wire.endTransmission();

        AccelOffset.x = -.03;
        AccelOffset.y = 0;
        AccelOffset.z = -0.11;

        a_ref.set(0, 0, 1.0f);
    }
    void read_gyro() {  //MPU6050 default address is 0x68
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x43);                                                //Register 67-72 - Gyroscope Measurements
        Wire.endTransmission();
        Wire.requestFrom(MPU_ADDR, 6);
        Vector3i Gyro;
        Gyro.x = Wire.read() << 8 | Wire.read();                  //67 GYRO_XOUT[15:8] and 68 GYRO_XOUT[7:0]
        Gyro.y = Wire.read() << 8 | Wire.read();                  //67 GYRO_XOUT[15:8] and 68 GYRO_XOUT[7:0]
        Gyro.z = Wire.read() << 8 | Wire.read();                  //67 GYRO_XOUT[15:8] and 68 GYRO_XOUT[7:0]
        GyroRate = Gyro.tofloat() / 65.536 *AP_DEG_TO_RAD;  //integer to deg/sec  /////////////////////////////////work with int instead?
        GyroRate.rotate(TO_NED_FRAME);
        GyroRate -= GyroOffset;
    }

    void read_accel() {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x3B);  //Register 59-64 - Accelerometer Measurements
        Wire.endTransmission();
        Wire.requestFrom(MPU_ADDR, 6);
        Vector3i AccLSB;
        AccLSB.x = -(Wire.read() << 8 | Wire.read());  //59 ACCEL_XOUT[15:8] and 60 ACCEL_XOUT[7:0]
        AccLSB.y = -(Wire.read() << 8 | Wire.read());  //61 ACCEL_YOUT[15:8] and 62 ACCEL_YOUT[7:0]
        AccLSB.z = -(Wire.read() << 8 | Wire.read());  //63 ACCEL_ZOUT[15:8] and 64 ACCEL_ZOUT[7:0]
        // to fix sign to MPU6050 frame negative stored

        AccelBody = AccLSB.tofloat()/4096 + AccelOffset;
        AccelBody.rotate(TO_NED_FRAME);
        UnitAccelBody = AccelBody.normalized();
        // acc_x = (float)AccXLSB / 4096 + AccXOffset;
        // acc_y = (float)AccYLSB / 4096 + AccYOffset;
        // acc_z = (float)AccZLSB / 4096 + AccZOffset;
// #if TP_ESTIMATOR == TP_BKF
//     
    }
    
    void calibrate_gyro(){
        GyroOffset.zero();
        Vector3f temp_sum; //since offsets are used in read sensor functions
        for (int i = 0; i < 2000; i++) {
            read_gyro();
            temp_sum += GyroRate;
            delay(1);
        }
        GyroOffset = temp_sum/2000;
    }
    // do this without serial inside the function??
    /*
    void calibrate_accel() {
        Serial.println("Calibrating Accelerometer");
        AccXOffset = AccYOffset = AccZOffset = 0;
        float sum = 0.0, X_Offset = 0, Y_Offset = 0, Z_Offset = 0;

        // Calibrate Z
        Serial.println("Calibrate Z: Lay Flat");
        delay(2000);
        Serial.println("Recording...");
        sum = 0;
        for (int i = 0; i < 2000; i++) {
            read_accel();
            sum += acc_z;
            delay(1);
        }
        Z_Offset = 1 - sum / 2000;
        Serial.println(Z_Offset);
        delay(2000);

        // Calibrate X
        Serial.println("Calibrate X: Pitch Up 90deg");
        delay(2000);
        Serial.println("Recording...");
        sum = 0;
        for (int i = 0; i < 2000; i++) {
            read_accel();
            sum += acc_x;
            delay(1);
        }
        X_Offset = 1 - sum / 2000;
        Serial.println(X_Offset);
        delay(2000);

        // Calibrate Y
        Serial.println("Calibrate Y: Roll Right 90deg");
        delay(2000);
        Serial.println("Recording...");
        sum = 0;
        for (int i = 0; i < 2000; i++) {
            read_accel();
            sum += acc_y;
            delay(1);
        }
        Y_Offset = 1 - sum / 2000;
        Serial.println(Y_Offset);
        delay(2000);
        Serial.println("Accelerometer Calibration Done!");
        delay(2000);
    }
    */

};
