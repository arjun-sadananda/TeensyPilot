#pragma once

#include "vector3.h"
#include "quaternion.h"
#include "TP_Sense\TP_MPU6050.h"
#include "TP_Sense\TP_Mag5883.h"
#include "TP_Sense\TP_BMP280.h"

#include "TP_Sense\TP_LSM9DS1.h"

#include <Arduino.h>
#include "Wire.h"
// #include "AP_Math.h"


#include "TP_StateEstimate\TP_BKF.h"
#include "TP_StateEstimate\TP_TRIAD.h"
#include "TP_StateEstimate\TP_EKF.h"
#include "TP_StateEstimate\TP_MEKF.h"
#include "TP_StateEstimate\TP_MEKF2.h"

#define BKF 0           // old stuff
#define TRIAD 1         
#define EKF 2           // Not working
#define MEKF_acc 3
#define MEKF_mag 4
#define MEKF2 5
#define MEKF2_TRIAD 6
#define ALL_ESTIMATORS 7
#define MEKF2_COMPARE 8

#define ESTIMATOR MEKF2_TRIAD

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
    Quaternion q;
    Vector3f omega;

#if ESTIMATOR == BKF
    TP_BKF tp_bkf;
#endif
#if ESTIMATOR == TRIAD || ESTIMATOR == ALL_ESTIMATORS
    TP_TRIAD tp_triad;
#endif
#if ESTIMATOR == EKF
    TP_EKF tp_ekf;
#endif
#if ESTIMATOR == MEKF_acc || ESTIMATOR == MEKF_mag || ESTIMATOR == ALL_ESTIMATORS
    TP_MEKF tp_mekf;
#endif
#if ESTIMATOR == MEKF2 || ESTIMATOR == ALL_ESTIMATORS || ESTIMATOR == MEKF2_COMPARE
    TP_MEKF2 tp_mekf2;
#endif
#if ESTIMATOR == MEKF2_TRIAD || ESTIMATOR == ALL_ESTIMATORS || ESTIMATOR == MEKF2_COMPARE
    TP_MEKF2 tp_mekf2_triad;
#endif
#if ESTIMATOR == MEKF2_COMPARE
    TP_MEKF2 tp_mekf2_acc;
#endif
    void init_sensors(){
#if SENSORS == MPU_QMC
        Wire.begin();
        Wire.setClock(400000);
        delay(1000);
        mpu.mpu_setup();
        mag.mag_setup();
        a_ref.set(mpu.a_ref.x, mpu.a_ref.y, mpu.a_ref.z);
        m_ref.set(mag.m_ref.x, mag.m_ref.y, mag.m_ref.z);
#elif SENSORS == LSM9D
    #if LSM9DS1_use_SPI
        // CSAG: 38
        // CSM:  37
        // alternate MISO1: 39
        pinMode (LSM_CSAG_pin, OUTPUT);
        pinMode (LSM_CSM_pin, OUTPUT);
        
        digitalWrite(LSM_CSAG_pin, HIGH);
        digitalWrite(LSM_CSM_pin, HIGH);
        SPI1.setMISO(39);
        // SPI1.setCS(38);
        SPI1.begin();
        // SPI.begin();
    #else
        Wire.begin();
        Wire.setClock(400000);
        delay(1000);
    #endif
        lsm9ds1.sensors_setup();
#endif
        delay(1000); 
    }

    void calibrate_sensors(){
#if SENSORS == MPU_QMC
        a_ref.set(mpu.a_ref.x, mpu.a_ref.y, mpu.a_ref.z);
        m_ref.set(mag.m_ref.x, mag.m_ref.y, mag.m_ref.z);
#elif SENSORS == LSM9D
        lsm9ds1.calibrate_gyro();
        lsm9ds1.set_a_ref();
        a_ref.set(lsm9ds1.a_ref.x, lsm9ds1.a_ref.y, lsm9ds1.a_ref.z);
        lsm9ds1.set_m_ref();
        m_ref.set(lsm9ds1.m_ref.x, lsm9ds1.m_ref.y, lsm9ds1.m_ref.z);
#endif
    }
    /*
        Initialise Estimators

        Calibrate Gyro
        Set m_ref and a_ref
        Initialise Estimators
    */
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
#endif
#if ESTIMATOR == MEKF_acc || ESTIMATOR == ALL_ESTIMATORS
        tp_mekf.init_estimator(a_ref);
#endif
#if ESTIMATOR == MEKF_mag 
        // mpu.calibrate_gyro();
        // mag.set_m_ref();
        tp_mekf.init_estimator(m_ref);
#endif
#if ESTIMATOR == MEKF2 || ESTIMATOR == ALL_ESTIMATORS || ESTIMATOR == MEKF2_COMPARE
        // mpu.calibrate_gyro();
        // mag.set_m_ref();
        tp_mekf2.init_estimator(a_ref, m_ref, false, 1.0e-3);
#endif 
#if ESTIMATOR == MEKF2_TRIAD || ESTIMATOR == ALL_ESTIMATORS || ESTIMATOR == MEKF2_COMPARE
        // mpu.calibrate_gyro();
        // mag.set_m_ref();
        tp_mekf2_triad.init_estimator(a_ref, m_ref, true, 1.0e-3);
#endif
#if ESTIMATOR == MEKF2_COMPARE
        // mpu.calibrate_gyro();
        // mag.set_m_ref();
        tp_mekf2_acc.init_estimator(a_ref, m_ref, false, 5.0e-3);
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
        // if (ESTIMATOR == MEKF_mag || ESTIMATOR == MEKF2 || ESTIMATOR == TRIAD || ESTIMATOR == BKF){
        mag.read_mag();
        UnitMagVect.set(mag.UnitMagVect.x, mag.UnitMagVect.y, mag.UnitMagVect.z);
        // }
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
#endif
#if ESTIMATOR == TRIAD || ESTIMATOR == ALL_ESTIMATORS
        tp_triad.estimate_attitude(UnitMagVect, UnitAccVect);
        q.from_rotation_matrix(tp_triad.DCM);
#endif
#if ESTIMATOR == EKF
        tp_ekf.estimate_attitude(mag.UnitMagVect, mpu.UnitAccVect, mpu.GyroRate, dt);
#endif
#if ESTIMATOR == MEKF_acc || ESTIMATOR == ALL_ESTIMATORS
        tp_mekf.estimate_attitude(UnitAccVect, GyroRate, dt);
        q = tp_mekf.get_q();
#endif
#if ESTIMATOR == MEKF_mag
        tp_mekf.estimate_attitude(UnitMagVect, GyroRate, dt);
#endif
#if ESTIMATOR == MEKF2 || ESTIMATOR == ALL_ESTIMATORS || ESTIMATOR == MEKF2_COMPARE
        tp_mekf2.estimate_attitude(UnitMagVect, UnitAccVect, GyroRate, dt);
        q = tp_mekf2.get_q();
#endif
#if ESTIMATOR == MEKF2_TRIAD || ESTIMATOR == ALL_ESTIMATORS || ESTIMATOR == MEKF2_COMPARE
        tp_mekf2_triad.estimate_attitude(UnitMagVect, UnitAccVect, GyroRate, dt);
        q = tp_mekf2_triad.get_q();
        omega = tp_mekf2_triad.get_omega();
#endif
#if ESTIMATOR == MEKF2_COMPARE
        tp_mekf2_acc.estimate_attitude(UnitMagVect, UnitAccVect, GyroRate, dt);
#endif
    }
};
