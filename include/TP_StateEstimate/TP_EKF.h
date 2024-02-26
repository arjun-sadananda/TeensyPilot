#pragma once

#include "TP_vector3.h"
#include "TP_MPU6050.h"
#include "TP_Mag5883.h"
#include "TP_BMP280.h"
#include <Arduino.h>
#include "Wire.h"
#include <BasicLinearAlgebra.h>


using namespace BLA;

class TP_EKF
{
private:
    float Ts = 0.005;

    BLA::Matrix<2,1> S = {0, 0}; 
    BLA::Matrix<1,1> Acc; BLA::Matrix<1,1> M;
    BLA::Matrix<2,2> F = {1, 0.004,
                        0, 1};  
    BLA::Matrix<2,1> G = {0.5*0.004*0.004,
                        0.004}; //S_dot = F S + G Acc
    BLA::Matrix<1,2> H = {1, 0}; //M = H S
    BLA::Matrix<2,2> P = {0, 0, 0, 0};
    BLA::Matrix<2,2> Q = G * ~G*10*10;
    BLA::Matrix<1,1> R = {30*30};
    BLA::Matrix<2,2> I = {1, 0, 0, 1};
    BLA::Matrix<2,1> K; 
    BLA::Matrix<1,1> L; 
protected:
public:
    MPU6050 mpu;
    Mag5883 mag;
    BMP280 baro;
    float var_roll_k = 0;                // Variance of state theta
    float var_pitch_k = 0;

    Vector3<float> attitude_euler; // Initialise state
    float altitude, velocity_z;

    TP_EKF(){}
    // TP_BKF(MPU6050 mpu, Mag5883 mag, BMP280 baro)
    // : mpu(mpu)
    // , mag(mag)
    // , baro(baro){}
    // TP_BKF(MPU6050 mpu, Mag5883 mag, BMP280 baro){
    //     mpu = mpu;
    //     mag = mag;
    //     baro = baro;
    // }

    void init_sensors(){
        mpu.mpu_setup();
        mag.mag_setup();
        baro.baro_setup();

        delay(250); 
    }
    
    void attitude_estimate(){
        mpu.read_gyro();

        attitude_euler.z = attitude_euler.z + Ts * mpu.GyroRate.z;
        // ******************************** Predict ********************************
        //x(k+1|k)          = A x(k|k)            + B  * u(k)
        attitude_euler.x = attitude_euler.x + Ts * mpu.GyroRate.x;
        attitude_euler.y = attitude_euler.y + Ts * mpu.GyroRate.y;
        //cov(k+1|k)        = A cov(k|k)          + sigma_process_noise (can contain drift)
        var_roll_k = var_roll_k + Ts * Ts * std_dev_gyro * std_dev_gyro;
        // var_pitch_k = var_pitch_k + Ts * Ts * std_dev_gyro * std_dev_gyro;

        // ******************************** Update *********************************
        mpu.read_accel();
        // KalmanGain = [cov(t+1|t)*C^T (C*cov(t+1|t)*C^T + sigma_measurement)^-1]
        //x(k+1|k+1)        = x(k+1|k) +  * (y(k+1) - C x(k+1|k))
        //cov(k+1|k+1)      = (1-K_k*C)      * cov(k+1|k)
        static float KalmanGain = var_roll_k * 1 / (1 * var_roll_k + std_dev_accel * std_dev_accel);
        attitude_euler.x = attitude_euler.x + KalmanGain * (mpu.accel_roll - attitude_euler.x);
        var_roll_k = (1 - KalmanGain) * var_roll_k;

        // KalmanGain = var_pitch_k * 1 / (1 * var_pitch_k + std_dev_accel * std_dev_accel);
        attitude_euler.y = attitude_euler.y + KalmanGain * (mpu.accel_pitch - attitude_euler.y);
        // var_pitch_k = (1 - KalmanGain) * var_pitch_k;
        //**************************************************************************
    }

    float get_vert_accel(){
        //Use linear ALgebra here...
        float AccelZ_inertial = - sin(attitude_euler.y*(3.142/180)) * mpu.AccelBody.x 
                        + cos(attitude_euler.y*(3.142/180)) * sin(attitude_euler.x*(3.142/180)) * mpu.AccelBody.y
                        + cos(attitude_euler.y*(3.142/180)) * cos(attitude_euler.x*(3.142/180)) * mpu.AccelBody.z;
        AccelZ_inertial = (AccelZ_inertial-1)*9.81*100;
        return AccelZ_inertial;
    }

    void vert_state_estimate(){
        // ******************************** Predict ******************************
        //read_accel() is already done
        Acc = {get_vert_accel()};
        S = F*S    + G*Acc;
        P = F*P*~F + Q;

        // ******************************** Update *********************************
        // Measure and Find Kalman Gain
        baro.read_baro();
        M = {baro.get_alt()};
        L = H*P*~H + R;
        K = P*~H*Invert(L);
        S = S + K*(M-H*S);
        P = (I-K*H)*P;

        altitude=S(0,0); 
        velocity_z=S(1,0); 
        //**************************************************************************
    }
};
