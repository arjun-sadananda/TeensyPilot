#pragma once

#include "vector3.h"
#include "quaternion.h"
#include "TP_Sense\TP_MPU6050.h"
#include "TP_Sense\TP_Mag5883.h"
#include "TP_Sense\TP_BMP280.h"

#include "TP_Sense\TP_LSM9DS1.h"

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

#define ESTIMATOR MEKF_acc

#define MPU_QMC 0
#define LSM9D 1 

#define SENSORS LSM9D

class TP_ESTIMATOR
{
private:
protected:
public:
    double prev_t, dt;
    // covariance matrix
#if SENSORS == MPU_QMC
    MPU6050 mpu;
    Mag5883 mag;
#elif SENSORS == LSM9D
    LSM9DS1 lsm9ds1;
#endif

    Vector3f MagRaw;
    Vector3f GyroRate, UnitAccVect, UnitMagVect;
    Vector3f a_ref, m_ref;

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
#if SENSORS == MPU_QMC
        mpu.mpu_setup();
        mag.mag_setup();
        a_ref.set(mpu.a_ref.x, mpu.a_ref.y, mpu.a_ref.z);
        m_ref.set(mag.m_ref.x, mag.m_ref.y, mag.m_ref.z);
#elif SENSORS == LSM9D
        lsm9ds1.sensors_setup();
        lsm9ds1.calibrate_gyro();
        lsm9ds1.set_a_ref();
        a_ref.set(lsm9ds1.a_ref.x, lsm9ds1.a_ref.y, lsm9ds1.a_ref.z);
        lsm9ds1.set_m_ref();
        m_ref.set(lsm9ds1.m_ref.x, lsm9ds1.m_ref.y, lsm9ds1.m_ref.z);
#endif
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
        tp_mekf.init_estimator(a_ref);
#elif ESTIMATOR == MEKF_mag
        // mpu.calibrate_gyro();
        // mag.set_m_ref();
        tp_mekf.init_estimator(m_ref);
#elif ESTIMATOR == MEKF2
        // mpu.calibrate_gyro();
        // mag.set_m_ref();
        tp_mekf2.init_estimator(a_ref, m_ref);
#endif
    }

    void read_sensors(){
#if SENSORS == MPU_QMC
        if(ESTIMATOR != TRIAD){
                mpu.read_gyro();
                GyroRate = mpu.GyroRate;
        }
        mpu.read_accel();
        UnitAccVect.set(mpu.UnitAccVect.x, mpu.UnitAccVect.y, mpu.UnitAccVect.z);
        if (ESTIMATOR == MEKF_mag || ESTIMATOR == MEKF2 || ESTIMATOR == TRIAD || ESTIMATOR == BKF){
                mag.read_mag();
                UnitMagVect.set(mag.UnitMagVect.x, mag.UnitMagVect.y, mag.UnitMagVect.z);
        }
        MagRaw.set(mag.MagRaw.x, mag.MagRaw.y, mag.MagRaw.z);
#elif SENSORS == LSM9D
        lsm9ds1.read_sensors();
        GyroRate = lsm9ds1.GyroRate;
        UnitAccVect.set(lsm9ds1.UnitAccVect.x, lsm9ds1.UnitAccVect.y, lsm9ds1.UnitAccVect.z);
        UnitMagVect.set(lsm9ds1.UnitMagVect.x, lsm9ds1.UnitMagVect.y, lsm9ds1.UnitMagVect.z);
        MagRaw.set(lsm9ds1.MagRaw.x, lsm9ds1.MagRaw.y, lsm9ds1.MagRaw.z);
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
        tp_bkf.estimate_attitude(mag.UnitMagVect, mpu.UnitAccVect, mpu.GyroRate, dt);
#elif ESTIMATOR == TRIAD
        tp_triad.estimate_attitude(UnitMagVect, UnitAccVect);
#elif ESTIMATOR == EKF
        tp_ekf.estimate_attitude(mag.UnitMagVect, mpu.UnitAccVect, mpu.GyroRate, dt);
#elif ESTIMATOR == MEKF_acc
        tp_mekf.estimate_attitude(UnitAccVect, GyroRate, dt);
#elif ESTIMATOR == MEKF_mag
        tp_mekf.estimate_attitude(UnitMagVect, GyroRate, dt);
#elif ESTIMATOR == MEKF2
        tp_mekf2.estimate_attitude(UnitMagVect, UnitAccVect, GyroRate, dt);
#endif
        
    }
};
