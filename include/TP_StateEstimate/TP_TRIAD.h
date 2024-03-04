#pragma once

#include "vector3.h"
#include "TP_Sense\TP_MPU6050.h"
#include "TP_Sense\TP_Mag5883.h"
#include "TP_Sense\TP_BMP280.h"
#include <Arduino.h>
#include "Wire.h"
#include <BasicLinearAlgebra.h>
#include "quaternion.h"
#include "matrix3.h"

using namespace BLA;

class TP_TRIAD

{
private:
protected:
public:
    MPU6050 mpu;
    Mag5883 mag;
    Matrix3f DCM;

    TP_TRIAD(){}

    void init_sensors(){
        mpu.mpu_setup();
        mag.mag_setup();
        delay(250); 
    }

    void estimate(){

        mpu.read_accel();
        mag.read_mag();

        static Vector3f c2, a, m;
        m = mag.UnitMagVect;
        a = mpu.UnitAccelBody;
        c2 = (a%m).normalized();
        DCM(c2%a, c2, a);
        //North(compass-ish)   East  gravity(Down)
    }
};