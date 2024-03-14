#pragma once

#include "vector3.h"
#include <Arduino.h>
#include "Wire.h"

class TP_BKF
{
private:

protected:
public:
    float var_roll_k = 0;                // Variance of state theta
    float var_pitch_k = 0;
    float std_dev_gyro, std_dev_accel;

    Vector3<float> attitude_euler; // Initialise state
    float altitude, velocity_z;

    
    float accel_roll, accel_pitch, mag_yaw; // Measurement from Accel after trignometry
    float AccelZ_inertial; 

    TP_BKF(){
        std_dev_gyro = 4*DEG_TO_RAD;     // 4deg/sec (try 1) 0.0698132rad/sec
        std_dev_accel = 3*DEG_TO_RAD;
    }
    // TP_BKF(MPU6050 mpu, Mag5883 mag, BMP280 baro)
    // : mpu(mpu)
    // , mag(mag)
    // , baro(baro){}
    // TP_BKF(MPU6050 mpu, Mag5883 mag, BMP280 baro){
    //     mpu = mpu;
    //     mag = mag;
    //     baro = baro;
    // }
    void estimate_attitude(Vector3f mag, Vector3f acc, Vector3f gyro, const double dt){
        
        // ******************************** Predict ********************************
        //x(k+1|k)          = A x(k|k)            + B  * u(k)
        attitude_euler.x = attitude_euler.x + dt * gyro.x;
        attitude_euler.y = attitude_euler.y + dt * gyro.y;
        attitude_euler.z = attitude_euler.z + dt * gyro.z;
        //cov(k+1|k)        = A cov(k|k)          + sigma_process_noise (can contain drift)
        var_roll_k = var_roll_k + dt * dt * std_dev_gyro * std_dev_gyro;
        // var_pitch_k = var_pitch_k + Ts * Ts * std_dev_gyro * std_dev_gyro;

        // ******************************** Update *********************************
        accel_roll = atan2(acc.y, acc.z); 
                            // sqrt(mpu.AccelBody.x * mpu.AccelBody.x + mpu.AccelBody.z * mpu.AccelBody.z));// * 1 / (3.142 / 180);  /////////////deg conversion unneccesry?
        accel_pitch = atan2(-acc.x, sqrt(acc.y * acc.y + acc.z * acc.z));// * 1 / (3.142 / 180);
        static float mx, my, mz;
        mx = mag.x;
        my = mag.y;
        mz = mag.z;
        // https://ahrs.readthedocs.io/en/latest/filters/complementary.html
        mag_yaw = atan2(mz*sin(accel_pitch)-my*cos(accel_pitch), 
                        mx*cos(accel_roll)+sin(accel_roll)*(my*sin(accel_pitch) + mz*cos(accel_pitch)));
        // KalmanGain = [cov(t+1|t)*C^T (C*cov(t+1|t)*C^T + sigma_measurement)^-1]
        // x(k+1|k+1)        = x(k+1|k) +  * (y(k+1) - C x(k+1|k))
        // cov(k+1|k+1)      = (1-K_k*C)      * cov(k+1|k)
        static float KalmanGain = var_roll_k * 1 / (1 * var_roll_k + std_dev_accel * std_dev_accel);
        attitude_euler.x = attitude_euler.x + KalmanGain * (accel_roll - attitude_euler.x);
        attitude_euler.y = attitude_euler.y + KalmanGain * (accel_pitch - attitude_euler.y);
        var_roll_k = (1 - KalmanGain) * var_roll_k;

        // KalmanGain = var_pitch_k * 1 / (1 * var_pitch_k + std_dev_accel * std_dev_accel);
        // var_pitch_k = (1 - KalmanGain) * var_pitch_k;

        // attitude_euler.z = attitude_euler.z + KalmanGain * (mag_yaw - attitude_euler.z);
        //**************************************************************************
    }

};
