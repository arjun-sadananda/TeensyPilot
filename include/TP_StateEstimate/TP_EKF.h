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
    
    const double Ts = 0.01;
    QuaternionD q;
    double P[16];    // state prediction and update state covriance
    
    double S[36], Ra, Rm, Rw, Qw;  // meaurement prediction covariance

    double K[24];
    
    Vector3d v_a, v_m; //Expected measurement and Innovation (Measurement Residual)
    Vector3d a_m, m_m, a_p, m_p;

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
        for(int k=0; k<16; k++) P[k] = 0.0;
        for(int k=0; k<16; k+=5) P[k] = 1.0e2;

        Qw = 1.0e1;
        Ra = 1.0e-3;//sq(mpu.std_dev_accel);
        Rm = 1.0e-3;//sq(mag.std_dev_mag);
        Rw = 1.0e-3;//mpu.std_dev_gyro;
    }

    void set_predicted_cov_mat_P(Vector3d &omega){
        //W_t = [-q_v^T; W_t2]
        static double q_w, k_Q, Q[16];
        // k_Q scalar to be multiplied by K*K^T
        // Q symmetric [4][4] process noise cov matrix
        static Vector3d q_v;
        static Vector3d w;

        q_w = q.q1;
        q_v.set(q.q2, q.q3, q.q4);

        k_Q = sq(Qw*Ts)/4.0f;
        // W_t_square.skew_from_vector(q_v);
        // W_t_square += q_w*eye3;

        // col = W_t_square*(-q_v);
        // W_t_square*=W_t_square;

        // Generate Process Noise Covariance Matrix
        // Q = k_Q*K*K^T
        Q[0] = k_Q * q_v.length_squared();  Q[4] = -k_Q * q_v.x * q_w;                      Q[8] = -k_Q * q_v.y * q_w;                      Q[12] = -k_Q * q_v.z * q_w;
        Q[1] = Q[4];                        Q[5] = k_Q * (sq(q_w)+sq(q_v.y)+sq(q_v.z));     Q[9] = -k_Q * q_v.x * q_v.y;                    Q[13] = -k_Q * q_v.z * q_v.x;
        Q[2] = Q[8];                        Q[6] = Q[9];                                    Q[10] = k_Q * (sq(q_w)+sq(q_v.z)+sq(q_v.x));    Q[14] = -k_Q * q_v.y * q_v.z;
        Q[3] = Q[12];                       Q[7] = Q[8];                                    Q[11] = Q[14];                                  Q[15] = k_Q * (sq(q_w)+sq(q_v.x)+sq(q_v.y));

        w = omega*0.5f*Ts;
        static double F[16];
        F[0] = 1.0;         F[4] = -w.x;        F[8]  = -w.y;       F[12] = -w.z;
        F[1] = w.x;         F[5] =  1.0;        F[9]  =  w.z;       F[13] = -w.y;
        F[2] = w.y;         F[6] = -w.z;        F[10] =  1.0;       F[14] =  w.x;
        F[3] = w.z;         F[7] =  w.y;        F[11] = -w.x;       F[15] =  1.0;

        static double S[16];
        // S = P*F'
        for(int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                double sum = 0.0;
                for(int k=0; k<4; k++) sum += P[i+k*4]*F[j+k*4];
                S[i*4+j] = sum;
            }
        }

        //P = F*P*F'
        for(int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                double sum = 0.0;
                for(int k=0; k<4; k++) sum += F[i+k*4]*S[k*4+j];
                P[i+j*4] = sum + Q[i+j*4];
            }
        }
    }

    void Cholesky( double* S ){
        // for each column
        for(int j=0; j<6; j++){
            double sum = 0.0;  //sum for the diagonal term
            // we first fill with 0.0 until diagonal
            for(int i=0; i<j; i++){
                S[i+j*6] = 0.0;
                //we can compute this sum at the same time
                sum += S[j+i*6]*S[j+i*6];
            }
            // now we compute the diagonal term
            S[j*7] = sqrt( S[j*7] - sum ); //S[j+j*m] = sqrt( S[j+j*m] - sum );
            // finally we compute the terms below the diagonal
            for(int i=j+1; i<6; i++){
                //first the sum
                sum = 0.0;
                for(char k=0; k<j; k++){
                    sum += S[i+k*6]*S[j+k*6];
                }
                //after the non-diagonal term
                S[i+j*6] = ( S[i+j*6] - sum )/S[j*7];
            }
        }//end j
        return;
    }

    void compute_K( double* S , double* M ){
        // we first compute the Cholesky decomposition for transform the system from  K*S = M  into K*L*L' = M
        Cholesky( S );
        
        double y[6];
        // then we take each pair of rows of K and M independently
        for(int i=0; i<4; i++){
            // first we solve (y*L' = M)
            for(int j=0; j<6; j++){
                double sum = 0.0;
                for(int k=0; k<j; k++){
                    sum += y[k]*S[j+k*6];
                }
                y[j] = ( M[i*6+j] - sum )/S[j*7];
            }
            // now we solve (Ki*L = y)
            for(int j=5; j>-1; j--){
                double sum = 0.0;
                for(int k=j+1; k<6; k++){
                    sum += M[i*6+k]*S[k+j*6];
                }
                M[i*6+j] = ( y[j] - sum )/S[j*7];
            }
        }
        for(int i=0; i<24; i++)
                K[i]=M[i];
        return;
    }

    void update(){
        /*  
            Declared as class members
            K[24] = P * H(q)^T * S^(-1)         4*4 x 4*6 x 6*6 = 4*6
            P[16] = (I - K*H(q_predict))P
        */
        
        static double H[24]; 
        static double M[24]; // P*H'
        static double S[36]; // H*P*H' + [noise covariance]
        const static Vector3d g(mpu.a_ref.x, mpu.a_ref.y, mpu.a_ref.z), r(mag.m_ref.x, mag.m_ref.y, mag.m_ref.z);

        static double q_w, q_x, q_y, q_z;
        q_w = q.q1; q_x = q.q2; q_y = q.q3; q_z = q.q4;

        H[0] =  g.x*q_w + g.y*q_z - g.z*q_y;     H[6]  = g.x*q_x + g.y*q_y + g.z*q_z;   H[12] = -g.x*q_y + g.y*q_x - g.z*q_w;  H[18] = -g.x*q_z + g.y*q_w + g.z*q_x;
        H[1] = -g.x*q_z + g.y*q_w + g.z*q_x;     H[7]  = g.x*q_y - g.y*q_x + g.z*q_w;   H[13] =  g.x*q_x + g.y*q_y + g.z*q_z;  H[19] = -g.x*q_w - g.y*q_z + g.z*q_y;
        H[2] =  g.x*q_y - g.y*q_x + g.z*q_w;     H[8]  = g.x*q_z - g.y*q_w - g.z*q_x;   H[14] =  g.x*q_w + g.y*q_z - g.z*q_y;  H[20] =  g.x*q_x + g.y*q_y + g.z*q_z;
        H[3] =  r.x*q_w + r.y*q_z - r.z*q_y;     H[9]  = r.x*q_x + r.y*q_y + r.z*q_z;   H[15] = -r.x*q_y + r.y*q_x - r.z*q_w;  H[21] = -r.x*q_z + r.y*q_w + r.z*q_x;
        H[4] = -r.x*q_z + r.y*q_w + r.z*q_x;     H[10] = r.x*q_y - r.y*q_x + r.z*q_w;   H[16] =  r.x*q_x + r.y*q_y + r.z*q_z;  H[22] = -r.x*q_w - r.y*q_z + r.z*q_y;
        H[5] =  r.x*q_y - r.y*q_x + r.z*q_w;     H[11] = r.x*q_z - r.y*q_w - r.z*q_x;   H[17] =  r.x*q_w + r.y*q_z - r.z*q_y;  H[23] =  r.x*q_x + r.y*q_y + r.z*q_z;

        // S = H*P*H^T + R

        // M = P*H'
        // to do:
        for(int i=0; i<4; i++){
            for(int j=0; j<6; j++){
                double sum = 0.0;
                for(int k=0; k<4; k++)   
                    sum += P[i+k*4]*H[j+k*6];
                M[i*6+j] = sum;
            }
        }
        // S = H*M + noise
        for(int j=0; j<6; j++){
            for(int i=0; i<6; i++){
                double sum = 0.0;
                for(int k=0; k<4; k++) 
                    sum += H[i+k*6]*M[k*6+j];
                S[i+j*6] = sum;
            }
        }
        // S = H*P*H' + R
        S[0]  += Ra;
        S[7]  += Ra;
        S[14] += Ra;
        S[21] += Rm;
        S[28] += Rm;
        S[35] += Rm;

        /* 
        *   now we can compute the gain
        *   S = H*P*H' + R
        *   M = P*H'
        * 
        *   Solve K*S = M  to  K*L*L' = M
        *   K[24] = P*H' * S_inv
        */
        compute_K( S , M );  // now K is stored in not M -> K
        
        
        /*
        *   Update the state
        */

        v_a = a_m - a_p;
        v_m = m_p - m_p;
        double dy[] = { v_a.x, v_a.y, v_a.z, v_m.x, v_m.y, v_m.z };

        for(int i=0; i<4; i++){
            double sum = 0.0;
            for(int j=0; j<6; j++) sum += K[i*6+j]*dy[j];
            q[i] += sum;
        }

        q.normalize();

        //    P = (I - K*H(q_predict))P
        static double N[16]; // P*H'
        static double T[16]; // H*P*H' + [noise covariance]

        // the covariance matrix is updated in the chart centered in qp
        // T = -K*H      4x6 * 6x4
        for(int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                double sum = 0.0;
                for(int k=0; k<4; k++) sum -= K[i*6+k]*H[k+j*6];
                T[i*4+j] = sum;
            }
        }
        // T = I-K*H
        for(int k=0; k<16; k+=5) S[k] += 1.0;
        // M = (I - K*H)*P_predicted
        for(int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                double sum = 0.0;
                for(int k=0; k<4; k++) sum += T[i*4+k]*P[k+j*4];
                N[i+j*4] = sum;
            }
        }
        // Force symmetry, P = P_updated = .5(M+M');
        for(int i=0; i<4; i++){
            for(int j=0; j<4; j++) 
                P[i+j*4] = 0.5*( N[i+j*4] + N[j+i*4] );
        }

        return;
    }

    void attitude_estimate(){

        // q_t.rotate(mpu.GyroRate);
        static QuaternionD q_from_gyro_dt;

        // both of the above are the same thing
        
        // Q_t.process_noise_cov_mat(q_t, mpu.std_dev_gyro, Ts);
        // ------- Predict -------
        mpu.read_gyro();
        static Vector3d w_m;
        w_m.set(mpu.GyroRate.x, mpu.GyroRate.y, mpu.GyroRate.z);
        q_from_gyro_dt.from_angular_velocity(w_m, Ts);
        set_predicted_cov_mat_P(w_m);    // use last quaternion state for Q=sigma^2*W*W^T -> P = F_omega*P*F_omega^T + Q
        q *= q_from_gyro_dt;         //F.from_omega_dt(mpu.GyroRate, Ts); q = F*q;

        // ------- Update --------
        mpu.read_accel();
        mag.read_mag();

        a_m = mpu.UnitAccelBody.todouble();
        m_m = mag.UnitMagVect.todouble(); // these are float... hmm...

        a_p = mpu.a_ref.todouble();
        q.inverse().earth_to_body(a_p);
        m_p = mag.m_ref.todouble();
        q.inverse().earth_to_body(m_p);

        update();
    }
};
