#pragma once

#include "vector3.h"
#include "quaternion.h"
#include "TP_Sense\TP_MPU6050.h"
#include "TP_Sense\TP_Mag5883.h"
#include "TP_Sense\TP_BMP280.h"
#include <Arduino.h>
#include "Wire.h"
#include "AP_Math.h"


#include "TP_StateEstimate\TP_BKF.h"
#include "TP_StateEstimate\TP_TRIAD.h"
#include "TP_StateEstimate\TP_EKF.h"
#include "TP_StateEstimate\TP_MEKF.h"
#include "TP_StateEstimate\TP_MEKF2.h"

#define BKF 0
#define TRIAD 1 
#define EKF 2
#define MEKF_acc 3
#define MEKF_mag 4
#define MEKF2 5

#define ESTIMATOR MEKF2

class TP_ESTIMATOR
{
private:
protected:
public:
    double prev_t, dt;
    // covariance matrix
    MPU6050 mpu;
    Mag5883 mag;

#if ESTIMATOR == BKF
    TP_BKF tp_bkf;
#elif ESTIMATOR == TRIAD
    TP_TRIAD tp_triad;
#elif ESTIMATOR == EKF
    TP_EKF tp_ekf;
#elif ESTIMATOR == MEKF_acc || ESTIMATOR == MEKF_mag
    TP_MEKF tp_mekf;
#elif ESTIMATOR == MEKF2
    TP_MEKF2 tp_mekf2;
#endif

    void init_sensors(){
        mpu.mpu_setup();
        mag.mag_setup();
        delay(250); 
    }
    void init_estimator(){
#if ESTIMATOR == BKF
        mpu.calibrate_gyro();
        mag.set_m_ref();
        // tp_bkf.init_estimator();
#elif ESTIMATOR == TRIAD
        
#elif ESTIMATOR == EKF
        mpu.calibrate_gyro();
        mag.set_m_ref();
        tp_ekf.init_estimator(mpu.a_ref, mag.m_ref);
#elif ESTIMATOR == MEKF_acc
        mpu.calibrate_gyro();
        tp_mekf.init_estimator(mpu.a_ref);
#elif ESTIMATOR == MEKF_mag
        mpu.calibrate_gyro();
        mag.set_m_ref();
        tp_mekf.init_estimator(mag.m_ref);
#elif ESTIMATOR == MEKF2
        mpu.calibrate_gyro();
        mag.set_m_ref();
        tp_mekf2.init_estimator(mpu.a_ref, mag.m_ref);
#endif
    }

    void read_sensors(){
#if ESTIMATOR != TRIAD
        mpu.read_gyro();
#endif
        mpu.read_accel();
#if ESTIMATOR == MEKF_mag || ESTIMATOR == MEKF2 || ESTIMATOR == TRIAD || ESTIMATOR == BKF
        mag.read_mag();
#endif
    }
    // Method: updateIMU
    // method used to update the state information through an IMU measurement
    // inputs:
    //  am: measured acceleration (g)
    //  wm: measured angular velocity (rad/s)
    //  dt: time step from the last update (s)
    // outputs:
    void estimate_attitude(){
        dt = (micros()-prev_t)/1000000.0;
        prev_t = micros();
#if ESTIMATOR == BKF
        tp_bkf.estimate_attitude(mag.UnitMagVect, mpu.UnitAccelBody, mpu.GyroRate, dt);
#elif ESTIMATOR == TRIAD
        tp_triad.estimate_attitude(mag.UnitMagVect, mpu.UnitAccelBody);
#elif ESTIMATOR == EKF
        tp_ekf.estimate_attitude(mag.UnitMagVect, mpu.UnitAccelBody, mpu.GyroRate, dt);
#elif ESTIMATOR == MEKF_acc
        tp_mekf.estimate_attitude(mpu.UnitAccelBody, mpu.GyroRate, dt);
#elif ESTIMATOR == MEKF_mag
        tp_mekf.estimate_attitude(mag.UnitMagVect, mpu.GyroRate, dt);
#elif ESTIMATOR == MEKF2
        tp_mekf2.estimate_attitude(mag.UnitMagVect, mpu.UnitAccelBody, mpu.GyroRate, dt);
#endif
        
    }
};
