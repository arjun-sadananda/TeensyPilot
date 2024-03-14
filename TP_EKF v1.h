#pragma once

#include "vector3.h"
// #include "matrix4.h"
#include "quaternion.h"
#include "TP_Sense\TP_MPU6050.h"
#include "TP_Sense\TP_Mag5883.h"
#include "TP_Sense\TP_BMP280.h"
#include <Arduino.h>
#include "Wire.h"
// #include "matrixN.h"
#include "AP_Math.h"



#define GYRO_ONLY true

struct Vector4
{
    float v[4];
};

struct Matrix4x6
{
    Vector4 V1, V2, V3, V4, V5, V6;
};


class TP_EKF
{
private:
protected:
public:
    MPU6050 mpu;
    Mag5883 mag;
    
    const float Ts = 0.02;
    Quaternion q;
    float P[10];    // state prediction and update state covriance
    Matrix4x6 K; // columns of Kalman Gain
    
    float S[21], R[6];  // meaurement prediction covariance
    float sigma_gyro;
    Matrix4x6 PHt; //PH'

    
    Vector3f H_ug, H_ur, q_v;
    Matrix3f H1, H2;
    // F and H are static variables in the respective unctions

    //P_t 4x4 symmetric covariance matrix
    //  0   4   7   9
    //  4   1   5   8
    //  7   5   2   6
    //  9   8   6   3

    
    Vector3f y_a, y_m, v_a, v_m; //Expected measurement and Innovation (Measurement Residual)

    TP_EKF(){}

    void init_sensors(){
        mpu.mpu_setup();
        mag.mag_setup();
        delay(250); 
        // Introduce Gyro Only Mode
    }
    void init_estimator(){
        // memset(&P[0][0], 0, sizeof(P));
        // set P to identity
        P[0] = P[1] = P[2] = P[3] = 1.0f;
        P[4] = P[5] = P[6] = P[7] = P[8] = P[9] = 0.0f;

        
        R[0] = R[1] = R[2] = sq(mpu.std_dev_accel);
        R[3] = R[4] = R[5] = sq(mag.std_dev_mag);
        sigma_gyro = mpu.std_dev_gyro;
    }

    // void from_omega_dt(const Vector3f &omega, float dt){
    //     static Vector3f w;
    //     w = omega*0.5f*dt;   //sqrt(omega.length_squared()+1);
    //     F_t[0][0] = 1;        F_t[0][1] = -w[0];      F_t[0][2] = -w[1];     F_t[0][3] = -w[2];
    //     F_t[1][0] = w[0];     F_t[1][1] =  1;         F_t[1][2] =  w[2];     F_t[1][3] = -w[1];
    //     F_t[2][0] = w[1];     F_t[2][1] = -w[2];      F_t[2][2] =  1;        F_t[2][3] =  w[0];
    //     F_t[3][0] = w[2];     F_t[3][1] =  w[1];      F_t[3][2] = -w[0];     F_t[3][3] =  1;
    // }

    // template <typename T>
    // Quaternion Matrix4<T>::operator         *(const Quaternion &q) const{
    //     return Quaternion(  mat[0][0]*q.q1      + mat[0][1]*q.q2        + mat[0][2]*q.q3       + mat[0][3]*q.q4,
    //                         mat[1][0]*q.q1      + mat[1][1]*q.q2        + mat[1][2]*q.q3       + mat[1][3]*q.q4,
    //                         mat[2][0]*q.q1      + mat[2][1]*q.q2        + mat[2][2]*q.q3       + mat[2][3]*q.q4,
    //                         mat[3][0]*q.q1      + mat[3][1]*q.q2        + mat[3][2]*q.q3       + mat[3][3]*q.q4);
    // }

    Vector4 P_times(const Vector4 v){
        static Vector4 ret;
        ret.v[0] = P[0]*v.v[0] + P[4]*v.v[1] + P[7]*v.v[2] + P[9]*v.v[3];
        ret.v[1] = P[4]*v.v[0] + P[1]*v.v[1] + P[5]*v.v[2] + P[8]*v.v[3];
        ret.v[2] = P[7]*v.v[0] + P[5]*v.v[1] + P[2]*v.v[2] + P[6]*v.v[3];
        ret.v[3] = P[9]*v.v[0] + P[8]*v.v[1] + P[6]*v.v[2] + P[3]*v.v[3];

        return ret;
    }

    Vector4 P_times(const float w, const Vector3f& v){
        static Vector4 ret;
        ret.v[0] = P[0]*w + P[4]*v.x + P[7]*v.y + P[9]*v.z;
        ret.v[1] = P[4]*w + P[1]*v.x + P[5]*v.y + P[8]*v.z;
        ret.v[2] = P[7]*w + P[5]*v.x + P[2]*v.y + P[6]*v.z;
        ret.v[3] = P[9]*w + P[8]*v.x + P[6]*v.y + P[3]*v.z;

        return ret;
    }

    Vector4 PHt_times(const float x1, const float x2, const float x3, const float x4, const float x5, const float x6){
        static Vector4 ret;
        ret.v[0] = PHt.V1.v[0]*x1 + PHt.V2.v[0]*x2 + PHt.V3.v[0]*x3 + PHt.V4.v[0]*x4 + PHt.V5.v[0]*x5 + PHt.V6.v[0]*x6;
        ret.v[1] = PHt.V1.v[1]*x1 + PHt.V2.v[1]*x2 + PHt.V3.v[1]*x3 + PHt.V4.v[1]*x4 + PHt.V5.v[1]*x5 + PHt.V6.v[1]*x6;
        ret.v[2] = PHt.V1.v[2]*x1 + PHt.V2.v[2]*x2 + PHt.V3.v[2]*x3 + PHt.V4.v[2]*x4 + PHt.V5.v[2]*x5 + PHt.V6.v[2]*x6;
        ret.v[3] = PHt.V1.v[3]*x1 + PHt.V2.v[3]*x2 + PHt.V3.v[3]*x3 + PHt.V4.v[3]*x4 + PHt.V5.v[3]*x5 + PHt.V6.v[3]*x6;

        return ret;
    }
    // can be used for making K as well (PH^T)*cols of(S^-1)
    Vector4 K_times(const Vector3f v1, const Vector3f v2){
        static Vector4 ret;
        ret.v[0] = K.V1.v[0]*v1.x + K.V2.v[0]*v1.y + K.V3.v[0]*v1.z + K.V4.v[0]*v2.x + K.V5.v[0]*v2.y + K.V6.v[0]*v2.z;
        ret.v[1] = K.V1.v[1]*v1.x + K.V2.v[1]*v1.y + K.V3.v[1]*v1.z + K.V4.v[1]*v2.x + K.V5.v[1]*v2.y + K.V6.v[1]*v2.z;
        ret.v[2] = K.V1.v[2]*v1.x + K.V2.v[2]*v1.y + K.V3.v[2]*v1.z + K.V4.v[2]*v2.x + K.V5.v[2]*v2.y + K.V6.v[2]*v2.z;
        ret.v[3] = K.V1.v[3]*v1.x + K.V2.v[3]*v1.y + K.V3.v[3]*v1.z + K.V4.v[3]*v2.x + K.V5.v[3]*v2.y + K.V6.v[3]*v2.z;

        return ret;
    }
    inline float dot_product4(const Vector4 v, const Vector4 w){
        return v.v[0]*w.v[0] + v.v[1]*w.v[1] + v.v[2]*w.v[2] + v.v[3]*w.v[3];
    }
    
    inline float dot_product4(const float w, const Vector3f v, const Vector4 v2){
        return w*v2.v[0] + v[0]*v2.v[1] + v[1]*v2.v[2] + v[2]*v2.v[3];
    }

    void set_predicted_cov_mat_P(Vector3f &omega){
        //W_t = [-q_v^T; W_t2]
        static float q_w, k_Q, Q[10];
        // k_Q scalar to be multiplied by K*K^T
        // Q symmetric [4][4] process noise cov matrix
        static Vector3f q_v;
        static Vector3f w;

        q_w = q.q1;
        q_v.set(q.q2, q.q3, q.q4);

        k_Q = sq(sigma_gyro*Ts)/4.0f;
        // W_t_square.skew_from_vector(q_v);
        // W_t_square += q_w*eye3;

        // col = W_t_square*(-q_v);
        // W_t_square*=W_t_square;

        // Generate Process Noise Covariance Matrix
        // Q = k_Q*K*K^T
        Q[0] = k_Q * q_v.length_squared();
        Q[4] = -k_Q * q_v.x * q_w;
        Q[7] = -k_Q * q_v.y * q_w;
        Q[9] = -k_Q * q_v.z * q_w;

        Q[1] = k_Q * (sq(q_w)+sq(q_v.y)+sq(q_v.z));
        Q[2] = k_Q * (sq(q_w)+sq(q_v.z)+sq(q_v.x));
        Q[3] = k_Q * (sq(q_w)+sq(q_v.x)+sq(q_v.y));

        Q[5] = -k_Q * q_v.x * q_v.y;
        Q[8] = -k_Q * q_v.z * q_v.x;
        Q[6] = -k_Q * q_v.y * q_v.z;

        w = omega*0.5f*Ts;

        //P_t-1*F^T = P*F'
        static Vector4 Pv1, Pv2, Pv3, Pv4;
        static Vector4 v1, v2, v3, v4;    // Rows of F or columns of F^T 
        v1.v[0] = v2.v[1] = v3.v[2] = v4.v[3] = 1;
        v1.v[1] = v4.v[2] = -w.x;
        v1.v[2] = v2.v[3] = -w.y;
        v1.v[3] = v3.v[1] = -w.z;
        
        v2.v[0] = v3.v[3] = w.x;
        v3.v[0] = v4.v[1] = w.y;
        v4.v[0] = v2.v[2] = w.z;

        Pv1 = P_times(v1);
        Pv2 = P_times(v2);
        Pv3 = P_times(v3);
        Pv4 = P_times(v4);

        //P = F*P*F'
        P[0] = dot_product4(v1, Pv1) + Q[0];
        P[1] = dot_product4(v2, Pv2) + Q[1];
        P[2] = dot_product4(v3, Pv3) + Q[2];
        P[3] = dot_product4(v4, Pv4) + Q[3];

        P[4] = dot_product4(v1, Pv2) + Q[4];
        P[5] = dot_product4(v2, Pv3) + Q[5];
        P[6] = dot_product4(v3, Pv4) + Q[6];

        P[7] = dot_product4(v1, Pv3) + Q[7];
        P[8] = dot_product4(v2, Pv4) + Q[8];

        P[9] = dot_product4(v1, Pv4) + Q[9];
    }

    void update(){
        /*
            K = P * H(q)^T * S^(-1)
            P = (I - K*H(q_predict))P
        */
       // Fense Post Problem (Off by one error)
        static float q_w;
        static Matrix3f eye3(1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
        // predicted q
        q_w = q.q1;
        q_v.set(q.q2, q.q3, q.q4);

        // z = h(x); y = Hx; find H(q) 
        // H = [ug [ug + q_w.g]_hat + (q_v.g)I3 - g*q_v^T]
        //     [ur [ur + q_w.r]_hat + (q_v.r)I3 - r*q_v^T]
        H_ug = mpu.a_ref%q_v;
        H_ur = mag.m_ref%q_v;
        H1.skew_from_vector(H_ug + mpu.a_ref*q_w);
        H2.skew_from_vector(H_ur + mag.m_ref*q_w);
        H1 += eye3*q_v.dot(mpu.a_ref) - mpu.a_ref.mul_rowcol(q_v);
        H2 += eye3*q_v.dot(mag.m_ref) - mag.m_ref.mul_rowcol(q_v);

        // S = H*P*H^T + R

        // [Pv1 Pv2 Pv3 Pv4 Pv5 Pv6] = P*H'
        PHt.V1 = P_times(H_ug.x, H1.a);
        PHt.V2 = P_times(H_ug.y, H1.b);
        PHt.V3 = P_times(H_ug.z, H1.c);
        PHt.V4 = P_times(H_ur.x, H2.a);
        PHt.V5 = P_times(H_ur.y, H2.b);
        PHt.V6 = P_times(H_ur.z, H2.c);
        
        // Symmetric (Positive Definite) Matrix
        // S = H*P*H' + R
        S[0] = dot_product4(H_ug.x, H1.a, PHt.V1) + R[0];
        S[1] = dot_product4(H_ug.y, H1.b, PHt.V2) + R[1];
        S[2] = dot_product4(H_ug.z, H1.c, PHt.V3) + R[2];
        S[3] = dot_product4(H_ur.x, H2.a, PHt.V4) + R[3];
        S[4] = dot_product4(H_ur.y, H2.b, PHt.V5) + R[4];
        S[5] = dot_product4(H_ur.z, H2.c, PHt.V6) + R[5];

        S[6]  = dot_product4(H_ug.x, H1.a, PHt.V2);
        S[7]  = dot_product4(H_ug.y, H1.b, PHt.V3);
        S[8]  = dot_product4(H_ug.z, H1.c, PHt.V4);
        S[9]  = dot_product4(H_ur.x, H2.a, PHt.V5);
        S[10] = dot_product4(H_ur.y, H2.b, PHt.V6);

        S[11] = dot_product4(H_ug.x, H1.a, PHt.V3);
        S[12] = dot_product4(H_ug.y, H1.b, PHt.V4);
        S[13] = dot_product4(H_ug.z, H1.c, PHt.V5);
        S[14] = dot_product4(H_ur.x, H2.a, PHt.V6);

        S[15] = dot_product4(H_ug.x, H1.a, PHt.V4);
        S[16] = dot_product4(H_ug.y, H1.b, PHt.V5);
        S[17] = dot_product4(H_ug.z, H1.c, PHt.V6);

        S[18] = dot_product4(H_ug.x, H1.a, PHt.V5);
        S[19] = dot_product4(H_ug.y, H1.b, PHt.V6);

        S[20] = dot_product4(H_ug.x, H1.a, PHt.V6);

        // S = L*L^T  Cholesky decomposition
        // S_inv = (L^-1)^T * (L^-1)
        // K = [Pv1 Pv2 ... Pv6]*S_inv
        static float S_inv[21], L[21], L_inv[21]; // No need to create L array can reuse S

        // Cholesky-Banachiewicz Lower Triangular Matrix, Loop unrolling for speed
        // https://en.wikipedia.org/wiki/Cholesky_decomposition#The_Cholesky%E2%80%93Banachiewicz_and_Cholesky%E2%80%93Crout_algorithms
        // S = L*L'
        L[0]  = sqrt(S[0]);

        L[6]  = S[6]/L[0];
        L[1]  = sqrt(S[1] - sq(L[6]));

        L[11] = S[11]/L[0];
        L[7]  = (S[7] - L[11]*L[6])/L[1];
        L[2]  = sqrt(S[2] - sq(L[7]) - sq(L[11]));
        
        L[15] = S[15]/L[0];
        L[12] = (S[12] - L[15]*L[6])/L[1];
        L[8]  = (S[8]  - L[15]*L[11] - L[12]*L[7])/L[2];
        L[3]  = sqrt(S[3] - sq(L[8]) - sq(L[12]) - sq(L[15]));

        L[18] = S[18]/L[0];
        L[16] = (S[16] - L[18]*L[6])/L[1];
        L[13] = (S[13] - L[18]*L[11] - L[16]*L[7])/L[2];
        L[9]  = (S[9]  - L[18]*L[15] - L[16]*L[12] - L[13]*L[8])/L[3];
        L[4]  = sqrt(S[4] - sq(L[9]) - sq(L[13]) - sq(L[16]) - sq(L[18]));

        L[20] = S[20]/L[0];
        L[19] = (S[19] - L[20]*L[6])/L[1];
        L[17] = (S[17] - L[20]*L[11] - L[19]*L[7])/L[2];
        L[14] = (S[14] - L[20]*L[15] - L[19]*L[12] - L[17]*L[8])/L[3];
        L[10] = (S[10] - L[20]*L[18] - L[19]*L[16] - L[17]*L[13] - L[14]*L[9])/L[4];
        L[5]  = sqrt(S[5] - sq(L[10]) - sq(L[14]) - sq(L[17]) - sq(L[19]) - sq(L[20]));

        // thanks for the help :P https://cplusplus.com/forum/beginner/245146/
        L_inv[0] = 1.0/L[0];
        L_inv[1] = 1.0/L[1];
        L_inv[2] = 1.0/L[2];
        L_inv[3] = 1.0/L[3];
        L_inv[4] = 1.0/L[4];
        L_inv[5] = 1.0/L[5];

        L_inv[6]  = -(L[6] *L_inv[0])/L[1];
        L_inv[7]  = -(L[7] *L_inv[1])/L[2];
        L_inv[8]  = -(L[8] *L_inv[2])/L[3];
        L_inv[9]  = -(L[9] *L_inv[3])/L[4];
        L_inv[10] = -(L[10]*L_inv[4])/L[5];

        L_inv[11] = -(L[11]*L_inv[0] + L[7] *L_inv[6])/L[2];
        L_inv[12] = -(L[12]*L_inv[1] + L[8] *L_inv[7])/L[3];
        L_inv[13] = -(L[13]*L_inv[2] + L[9] *L_inv[8])/L[4];
        L_inv[14] = -(L[14]*L_inv[3] + L[10]*L_inv[9])/L[5];

        L_inv[15] = -(L[15]*L_inv[0] + L[12]*L_inv[6] + L[8] *L_inv[11])/L[3];
        L_inv[16] = -(L[16]*L_inv[1] + L[13]*L_inv[7] + L[9] *L_inv[12])/L[4];
        L_inv[17] = -(L[17]*L_inv[2] + L[14]*L_inv[8] + L[10]*L_inv[13])/L[5];
        
        L_inv[18] = -(L[18]*L_inv[0] + L[16]*L_inv[6] + L[13]*L_inv[11] + L[9] *L_inv[15])/L[4];
        L_inv[19] = -(L[19]*L_inv[1] + L[17]*L_inv[7] + L[14]*L_inv[12] + L[10]*L_inv[16])/L[5];

        L_inv[20] = -(L[20]*L_inv[0] + L[19]*L_inv[6] + L[17]*L_inv[11] + L[14]*L_inv[15] + L[10]*L_inv[18])/L[5];

        // S_inv = (L*L')^-1 = L_inv' * L_inv (Linear combination of columns of L_inv' by scalar entries in L_inv)
        S_inv[20] = L_inv[5]*L_inv[20];
        S_inv[19] = L_inv[5]*L_inv[19];
        S_inv[17] = L_inv[5]*L_inv[17];
        S_inv[14] = L_inv[5]*L_inv[14];
        S_inv[10] = L_inv[5]*L_inv[10];
        S_inv[5]  = L_inv[5]*L_inv[5];

        S_inv[18] = L_inv[4]*L_inv[18] + L_inv[10]*L_inv[20];
        S_inv[16] = L_inv[4]*L_inv[16] + L_inv[10]*L_inv[19];
        S_inv[13] = L_inv[4]*L_inv[13] + L_inv[10]*L_inv[17];
        S_inv[9]  = L_inv[4]*L_inv[9]  + L_inv[10]*L_inv[14];
        S_inv[4]  = L_inv[4]*L_inv[4]  + L_inv[10]*L_inv[10];
      //S_inv[10]'= L_inv[4]*0         + L_inv[10]*L_inv[5]; = S_inv[10] Checking Symmetric

        S_inv[15] = L_inv[3]*L_inv[15] + L_inv[9]*L_inv[18] + L_inv[14]*L_inv[20];
        S_inv[12] = L_inv[3]*L_inv[12] + L_inv[9]*L_inv[16] + L_inv[14]*L_inv[19];
        S_inv[8]  = L_inv[3]*L_inv[8]  + L_inv[9]*L_inv[13] + L_inv[14]*L_inv[17];
        S_inv[3]  = L_inv[3]*L_inv[3]  + L_inv[9]*L_inv[9]  + L_inv[14]*L_inv[14];

        S_inv[11] = L_inv[2]*L_inv[11] + L_inv[8]*L_inv[15] + L_inv[13]*L_inv[18] + L_inv[17]*L_inv[20];
        S_inv[7]  = L_inv[2]*L_inv[7]  + L_inv[8]*L_inv[12] + L_inv[13]*L_inv[16] + L_inv[17]*L_inv[19];
        S_inv[2]  = L_inv[2]*L_inv[2]  + L_inv[8]*L_inv[8]  + L_inv[13]*L_inv[13] + L_inv[17]*L_inv[17];

        S_inv[6]  = L_inv[1]*L_inv[6]  + L_inv[7]*L_inv[11] + L_inv[12]*L_inv[15] + L_inv[16]*L_inv[18] + L_inv[19]*L_inv[20];
        S_inv[1]  = L_inv[1]*L_inv[1]  + L_inv[7]*L_inv[7]  + L_inv[12]*L_inv[12] + L_inv[16]*L_inv[16] + L_inv[19]*L_inv[19];
        
        S_inv[0]  = sq(L_inv[0]) + sq(L_inv[6]) + sq(L_inv[11]) + sq(L_inv[15]) + sq(L_inv[18]) + sq(L_inv[20]);

        // K = P*H'(q)*S_inv = [Pv1 Pv2 ... Pv6]*S_inv
        K.V1 = PHt_times(S_inv[0] , S_inv[6] , S_inv[11], S_inv[15], S_inv[18], S_inv[20]);
        K.V2 = PHt_times(S_inv[6] , S_inv[1] , S_inv[7] , S_inv[12], S_inv[16], S_inv[19]);
        K.V3 = PHt_times(S_inv[11], S_inv[7] , S_inv[2] , S_inv[8] , S_inv[13], S_inv[17]);
        K.V4 = PHt_times(S_inv[15], S_inv[12], S_inv[8] , S_inv[3] , S_inv[9] , S_inv[14]);
        K.V5 = PHt_times(S_inv[18], S_inv[16], S_inv[13], S_inv[9] , S_inv[4] , S_inv[10]);
        K.V6 = PHt_times(S_inv[20], S_inv[19], S_inv[17], S_inv[14], S_inv[10], S_inv[5]);
        
        //    P = (I - K*H(q_predict))P
        static Vector4 KH[4]; //columns of K*H
        H1.transpose();
        H2.transpose();
        KH[0] = K_times(-H_ug, -H_ur);
        KH[1] = K_times(-H1.a, -H2.a);
        KH[2] = K_times(-H1.b, -H2.b);
        KH[3] = K_times(-H1.c, -H2.c);

        KH[0].v[0] +=1;
        KH[1].v[1] +=1;
        KH[2].v[2] +=1;
        KH[3].v[3] +=1;

        static Vector4 P_new[4];
        // P is symmetrix so P_times can be used
        // is I-Kh symmetric? i hope so.
        /// therefore double use of symmetricitutiyt
        P_new[0] = P_times(KH[0]);
        P_new[1] = P_times(KH[1]);
        P_new[2] = P_times(KH[2]);
        P_new[3] = P_times(KH[3]);

        P[0] = P_new[0].v[0];
        P[4] = P_new[0].v[1];
        P[7] = P_new[0].v[2];
        P[9] = P_new[0].v[3];
        
        P[1] = P_new[1].v[1];
        P[5] = P_new[1].v[2];
        P[8] = P_new[1].v[3];

        P[2] = P_new[2].v[2];
        P[6] = P_new[2].v[3];
        
        P[3] = P_new[3].v[3];
    }

    void attitude_estimate(){
        mpu.read_gyro();
        // q_t.rotate(mpu.GyroRate);
        static Quaternion q_from_gyro_dt;

        // both of the above are the same thing
        
        // Q_t.process_noise_cov_mat(q_t, mpu.std_dev_gyro, Ts);
        // ------- Predict -------
        q_from_gyro_dt.from_angular_velocity(mpu.GyroRate, Ts);
        set_predicted_cov_mat_P(mpu.GyroRate);    // use last quaternion state for Q=sigma^2*W*W^T -> P = F_omega*P*F_omega^T + Q
        q *= q_from_gyro_dt;         //F.from_omega_dt(mpu.GyroRate, Ts); q = F*q;

        // ------- Update --------
        mpu.read_accel();
        mag.read_mag();

        y_a = mpu.a_ref;
        // q.inverse().earth_to_body(y_a);
        y_m = mag.m_ref;
        // q.inverse().earth_to_body(y_m);

        update();
        
        Vector3f qv;
        qv.set(q.q2, q.q2, q.q3);
        y_a.x = H_ug.x*q.q1 + H1.a.dot(qv);
        y_a.y = H_ug.y*q.q1 + H1.b.dot(qv);
        y_a.z = H_ug.z*q.q1 + H1.c.dot(qv);
        y_m.x = H_ur.x*q.q1 + H1.a.dot(qv);
        y_m.y = H_ur.y*q.q1 + H1.b.dot(qv);
        y_m.z = H_ur.z*q.q1 + H1.c.dot(qv);

        v_a = mpu.UnitAccelBody - y_a;
        v_m = mag.UnitMagVect - y_m;
        
        // static Vector4 q_new;
        // q_new = K_times(v_a, v_m);
        // Quaternion q_del;
        // q_del.q1 = q_new.v[0];
        // q_del.q2 = q_new.v[1];
        // q_del.q3 = q_new.v[2];
        // q_del.q4 = q_new.v[3];
        // q *= q_del;

        // q.q1 = q.q1 + q_new.v[0];
        // q.q2 = q.q2 + q_new.v[1];
        // q.q3 = q.q3 + q_new.v[2];
        // q.q4 = q.q4 + q_new.v[3];

        // q_t = F_t*q_t;
        q.normalize();


        /*
            ----- Prediction -----
            q_t = self.f(q, g)                  # Predicted State
            F   = self.dfdq(g)                  # Linearized Fundamental Matrix

            W   = 0.5*self.Dt * np.r_[[-q[1:]], q[0]*np.identity(3) + skew(q[1:])]  # Jacobian W = df/dω
            Q_t = 0.5*self.Dt * self.g_noise * W@W.T    # Process Noise Covariance
            P_t = F@self.P@F.T + Q_t            # Predicted Covariance Matrix
            # ----- Correction -----
            y   = self.h(q_t)                   # Expected Measurement function
            v   = self.z - y                    # Innovation (Measurement Residual)
            H   = self.dhdq(q_t)                # Linearized Measurement Matrix
            S   = H@P_t@H.T + self.R            # Measurement Prediction Covariance
            K   = P_t@H.T@np.linalg.inv(S)      # Kalman Gain
            self.P = (np.identity(4) - K@H)@P_t
            self.q = q_t + K@v                  # Corrected state
            self.q /= np.linalg.norm(self.q)    
        */
    }
};
