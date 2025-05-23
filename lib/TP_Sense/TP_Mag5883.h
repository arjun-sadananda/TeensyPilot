#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "vector3.h"
#include "matrix3.h"

#define MAG_ADDR 0x0D //byte
#define MAG_CONTROL_REG_1 0x09

class Mag5883
{
private:
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
    Mag5883(){}
    void mag_setup() {  //MPU6050 default address is 0x68 MPU:Motion Processing Units
        Wire.beginTransmission(MAG_ADDR);
        Wire.write(0x0B);  // SET/RESET Period Register
        Wire.write(0x01);  // 
        Wire.endTransmission();

        Wire.beginTransmission(MAG_ADDR);
        Wire.write(0x09);  // Control Register
        Wire.write(0x1D);  //OverSampleRatio=512 RNG FullScale=8G OutputDataRate = 200Hz Mode = Continuous 
                        // 8Gauss = 800uT 2^16/(2*800)= 40.96 LSB/uT
                        // ----- LatLong = 19.1334° N, 72.9133° E
                        // 	Declination (+E|-W)  = Negligible                    -0.0477° or .05W  +- 0.30°
                        //  Inclination (+D|-U)  = 28.6536° +- 0.21°             ---- Important
                        //  Total Field          = 43,634.0 nT = sqrt(38.2903^2 + 20.923^2) uT = 43.634uT = 43.634*40.96 LSB ~= 1787LSB
                        //  Horizontal Intensity = 38,290.3 nT
                        //  Vertical Comp        = 20,923.0 nT
                        //  East Comp            = Negligible                    -31.8 nT +- 94 nT
                        //  bx
        Wire.endTransmission();

        // HardOffsetVect.set(281.3, 1002.3, -322.54);
        // HardOffsetVect.set(281.839179, 1227.446283, -497.300555);
        HardOffsetVect.set(-4.351436, 744.005033, -102.588960);

        // float temp[6] =  {1.29, 1.31, 1.28, 0.027, -0.0698, 0.01394};
        // float temp[6] =  {1.320546, 1.152157, 1.301622, 0.000088, -0.026157, 0.122299};
        float temp[6] =  {1.294398, 1.277933, 1.369600, 0.002373, -0.005930, -0.039244};
        SoftCalibMat.a.set(temp[0], temp[3], temp[4]);
        SoftCalibMat.b.set(temp[3], temp[1], temp[5]);
        SoftCalibMat.c.set(temp[4], temp[5], temp[2]);

        // m_ref.set(38290.3, 0, 20923.0); //NED
        // m_ref.normalize();
        set_m_ref();
    }
    void set_m_ref(){
        Vector3f sum;
        for(int i = 0; i<1000; i++){
            read_mag();
            sum += UnitMagVect;
        }
        m_ref = sum/=1000;
        // m_ref.set(38290.3, 0, 20923.0); //NED
        m_ref.normalize();
    }
    void read_mag(){
        // int mag_raw[3] = {0,0,0};
        Wire.beginTransmission(MAG_ADDR);
        Wire.write(0x00);
        int err = Wire.endTransmission();
        if(!err){
            Wire.requestFrom(MAG_ADDR, 6);
            MagRaw.x = (float)(int16_t)(Wire.read()|Wire.read()<<8);
            MagRaw.y = (float)(int16_t)(Wire.read()|Wire.read()<<8);
            MagRaw.z = (float)(int16_t)(Wire.read()|Wire.read()<<8);
        }
        //Calibration
        MagVect = SoftCalibMat*(MagRaw-HardOffsetVect);
        MagVect.rotate(TO_NED_FRAME);
        UnitMagVect = MagVect.normalized();
    }
};
