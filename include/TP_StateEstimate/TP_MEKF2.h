#pragma once

#include <Arduino.h>
#include "Wire.h"
// #include "matrixN.h"
// #include "AP_Math.h"

#include "SD.h"



#define GYRO_ONLY false

// Thanks to: https://github.com/PBernalPolo/test_MKF.git

class TP_MEKF2
{
private:
protected:
    // PRIVATE VARIABLES
    // quaternion describing the orientation (q1,q2,q3,q4)=(qx,qy,qz,qw) wrong order !
    // (rotation that transform vectors from the sensor reference frame, to the external reference frame)
    // double q[4];
    QuaternionD q;
    QuaternionD q_p;
    // angular velocity (rad/s)
    // double w[3];
    Vector3d w;
    // covariance matrix of the angular velocity noise (rad^2/s^3)
    double Qw;
    // covariance matrix of the acceleration noise (g^2)
    double Qa, Qm;
    // covariance matrix of the angular velocity measurement noise (rad^2/s^2)
    double Rw;
    // covariance matrix of the acceleration measurement noise (g^2)
    double Ra, Rm;
    // use or not the chart update
    boolean chartUpdate = false;

    // Quaternion quat;

public:
    // covariance matrix
    bool use_triad = false;
    double P[36];
    double K[54];
    Vector3d a_p, m_p;
    Vector3f a_m, m_m, w_m, a_ref, m_ref;
    double dy[9];
    double get_a_res_norm(){
        return sq(dy[0]) + sq(dy[1]) + sq(dy[2]);
    }
    double get_m_res_norm(){
        return sq(dy[3]) + sq(dy[4]) + sq(dy[5]);
    }
    TP_MEKF2(){
        q[0] = 1.0;   q[1] = 0.0;   q[2] = 0.0;   q[3] = 0.0;
        // it is necessary to set an angular velocity different from 0.0 to break the symmetry
        // otherwise, the MUKF could not converge (especially when we apply the "reset operation" with the RV chart)
        w[0] = 1.0e-10;   w[1] = 1.0e-10;   w[2] = 1.0e-10;
        
        for(int k=0; k<36; k++) P[k] = 0.0;
        for(int k=0; k<36; k+=7) P[k] = 1.0e2;
        P[2+2*6] = 1.0e-16;
        
        Qw = 1.0e1;
        Qa = 0;//1.0e-3;//0
        Qm = 0;//1.0e-3;//0
        Rw = 1.0e-2; // Gyro Measure Noise 1.0e-3
        Ra = 1.0e-2;
        Rm = 1.0e-2; // 10.0e-3 vs 1.0e-3
    }

    Quaternion get_q(){
        static Quaternion ret;
        // ret[0] = (float) q[3];
        for(int i=0; i<4; i++) ret[i] = (float) q[i];
        return ret;
    }
    Quaternion get_qp(){
        static Quaternion ret;
        // ret[0] = (float) q[3];
        for(int i=0; i<4; i++) ret[i] = (float) q_p[i];
        return ret;
    }
    Vector3f get_omega(){
        static Vector3f ret;
        // ret[0] = (float) q[3];
        ret[0] = (float) w[0];
        ret[1] = (float) w[1];
        ret[2] = (float) w[2];
        return ret;
    }
    
    void set_chartUpdate( boolean chartUpdateIn ){
        chartUpdate = chartUpdateIn;
    }
    
    void init_estimator(const Vector3f a_r, const Vector3f m_r, bool triad_on, double R_m = 1.0e-2){
        // Introduce Gyro Only Mode
        use_triad = triad_on;
        // R[0] = R[1] = R[2] = sq(mpu.std_dev_accel);
        // R[3] = R[4] = R[5] = sq(mag.std_dev_mag);
        // sigma_gyro = mpu.std_dev_gyro;
        // q[0] = 1.0;
        // q[1] = 0.0;
        // q[2] = 0.0;
        // q[3] = 0.0;
        a_ref = a_r;
        if (use_triad)
            m_ref = (a_r%m_r).normalized()%a_r;
        else
            m_ref = m_r;
            
        Rm = R_m; // 10.0e-3 vs 1.0e-3
    }


    void predict_covariance(const QuaternionD q_del, const double dt){
        // P in present (last) q-centered chart += Q
        P[0]  += Qw*dt*dt*dt/3;         P[18] -= Qw*dt*dt/2;
        P[7]  += Qw*dt*dt*dt/3;         P[25] -= Qw*dt*dt/2;
        P[14] += Qw*dt*dt*dt/3;         P[32] -= Qw*dt*dt/2;

        P[3]  -= Qw*dt*dt/2;            P[21] += Qw*dt;
        P[10] -= Qw*dt*dt/2;            P[28] += Qw*dt;
        P[17] -= Qw*dt*dt/2;            P[35] += Qw*dt;

        // F =  [ R'(del) I*dt ]
        //      [    0      I  ]
        static double F[36];
        F[0] = 1.0-2.0*q_del[2]*q_del[2]-2.0*q_del[3]*q_del[3];  F[6] = 2.0*(q_del[1]*q_del[2]+q_del[3]*q_del[0]);        F[12] = 2.0*(q_del[1]*q_del[3]-q_del[2]*q_del[0]);
        F[1] = 2.0*(q_del[1]*q_del[2]-q_del[3]*q_del[0]);        F[7] = 1.0-2.0*q_del[1]*q_del[1]-2.0*q_del[3]*q_del[3];  F[13] = 2.0*(q_del[2]*q_del[3]+q_del[1]*q_del[0]);
        F[2] = 2.0*(q_del[1]*q_del[3]+q_del[2]*q_del[0]);        F[8] = 2.0*(q_del[2]*q_del[3]-q_del[1]*q_del[0]);        F[14] = 1.0-2.0*q_del[1]*q_del[1]-2.0*q_del[2]*q_del[2];

                                                        F[18] = dt;     F[24] =0.0;     F[30] = 0.0;
                                                        F[19] = 0.0;    F[25] = dt;     F[31] = 0.0;
                                                        F[20] = 0.0;    F[26] = 0.0;    F[32] = dt;

        F[3] = 0.0;     F[9] = 0.0;     F[15] = 0.0;    F[21] = 1.0;    F[27] = 0.0;    F[33] = 0.0;
        F[4] = 0.0;     F[10] = 0.0;    F[16] = 0.0;    F[22] = 0.0;    F[28] = 1.0;    F[34] = 0.0;
        F[5] = 0.0;     F[11] = 0.0;    F[17] = 0.0;    F[23] = 0.0;    F[29] = 0.0;    F[35] = 1.0;
        
        // P = F*P*F' (= F*[P+Q]*F')
        static double S[36];
        // P*F'
        for(int i=0; i<6; i++){
            for(int j=0; j<6; j++){
                double sum = 0.0;
                for(int k=0; k<6; k++) sum += P[i+k*6]*F[j+k*6];
                S[i*6+j] = sum;
            }
        }
        for(int i=0; i<6; i++){
            for(int j=0; j<6; j++){
                double sum = 0.0;
                for(int k=0; k<6; k++) sum += F[i+k*6]*S[k*6+j];
                P[i+j*6] = sum;
            }
        }
            
    }
     // Method: updateIMU
  // method used to update the state information through an IMU measurement
  // inputs:
  //  am: measured acceleration (g)
  //  wm: measured angular velocity (rad/s)
  //  dt: time step from the last update (s)
  // outputs:
    void estimate_attitude(Vector3f mag, Vector3f acc, Vector3f gyro, const double dt){

        // compute the state prediction
        static QuaternionD q_del; // used twice for prediction and for update. delta q
        if( !w.is_zero() )
            q_del.from_angular_velocity(w,dt);
        else
            q_del.initialise();                 // No prediction just update!?

        q_p = q * q_del;

        // compute the covariance matrix for the state prediction
        predict_covariance(q_del, dt);

        /*
        *  we compute the measurement prediction
        */

        // Predicted Measurement
        // a_p = R'(q_pred)*[0 0 1]' = [0 0 1]*R'(q_pred)
        // a_p = q_p.gravity_vector();
        a_p = a_ref.todouble();
        m_p = m_ref.todouble();
        q_p.inverse().earth_to_body(a_p);
        q_p.inverse().earth_to_body(m_p);
        
        // H   = [ [a_p]x      0 ]
        //       [     0       I ]
        static double H[54]; 
        static double M[54]; // P*H'
        static double S[81]; // H*P*H' + [noise covariance]
        H[0] = 0.0;      H[9] = -m_p[2];   H[18] = m_p[1];   H[27] = 0.0;    H[36] = 0.0;    H[45] = 0.0;
        H[1] = m_p[2];   H[10] = 0.0;      H[19] = -m_p[0];  H[28] = 0.0;    H[37] = 0.0;    H[46] = 0.0;
        H[2] = -m_p[1];  H[11] = m_p[0];   H[20] = 0.0;      H[29] = 0.0;    H[38] = 0.0;    H[47] = 0.0;
        H[3] = 0.0;      H[12] = -a_p[2];  H[21] = a_p[1];   H[30] = 0.0;    H[39] = 0.0;    H[48] = 0.0;
        H[4] = a_p[2];   H[13] = 0.0;      H[22] = -a_p[0];  H[31] = 0.0;    H[40] = 0.0;    H[49] = 0.0;
        H[5] = -a_p[1];  H[14] = a_p[0];   H[23] = 0.0;      H[32] = 0.0;    H[41] = 0.0;    H[50] = 0.0;
        H[6] = 0.0;      H[15] = 0.0;      H[24] = 0.0;      H[33] = 1.0;    H[42] = 0.0;    H[51] = 0.0;
        H[7] = 0.0;      H[16] = 0.0;      H[25] = 0.0;      H[34] = 0.0;    H[43] = 1.0;    H[52] = 0.0;
        H[8] = 0.0;      H[17] = 0.0;      H[26] = 0.0;      H[35] = 0.0;    H[44] = 0.0;    H[53] = 1.0;
        
        // M = P*H'     6x9 = 6x6 * 6x9  === <- || * ===
        for(int i=0; i<6; i++){
            for(int j=0; j<9; j++){
                double sum = 0.0;
                for(int k=0; k<6; k++) 
                    sum += P[i+k*6]*H[j+k*9];
                M[i*9+j] = sum;
            }
        }
        // for(int i =0; i<54; i++){
        //     Serial.print(M[i]);
        //     Serial.print(" ");
        // }
        // S = H*M + noise   9x6 * 6x9  
        for(int j=0; j<9; j++){
            for(int i=0; i<9; i++){
                double sum = 0.0;
                for(int k=0; k<6; k++) 
                    sum += H[i+k*9]*M[k*9+j];
                S[i+j*9] = sum;
            }
        }
        

        // S = H*P*H' + [ R'*Qa*R + Ra*I , 0 ;  0 , Rw*I ]
        // Qa is not passed through rotaion trnsform!? (in original implementation as well)
        S[0]   += Qm + Rm;
        S[10]  += Qm + Rm;
        S[20]  += Qm + Rm;
        S[30]  += Qa + Ra;
        S[40]  += Qa + Ra;
        S[50]  += Qa + Ra;
        S[60]  += Rw;
        S[70]  += Rw;
        S[80]  += Rw;
        /* 
        *   now we can compute the gain
        */
        // for(int i =0; i<9; i++){
        //     Serial.print(S[i*10]);
        //     Serial.print(" ");
        // }
        compute_K( S , M );  // now K is stored in not M -> K
        /*
        *   Measure
        */
        m_m = mag;
        a_m = acc;
        w_m = gyro; // these are float... hmm...

        // TRIAD idea based preprocessing
        if (use_triad)
            m_m = (a_m%m_m).normalized()%a_m;

        /*
        *   Update the state in the chart
        */
        dy[0] = m_m[0]-m_p[0];
        dy[1] = m_m[1]-m_p[1]; 
        dy[2] = m_m[2]-m_p[2]; 
        dy[3] = a_m[0]-a_p[0]; 
        dy[4] = a_m[1]-a_p[1]; 
        dy[5] = a_m[2]-a_p[2]; 
        dy[6] = w_m[0]-w[0]; 
        dy[7] = w_m[1]-w[1];
        dy[8] = w_m[2]-w[2];
        // dy = measurements - predictions
        // dx = K*(dy); 
        double dx[6];
        for(int i=0; i<6; i++){
            double sum = 0.0;
            for(int j=0; j<9; j++) sum += K[i*9+j]*dy[j];
            dx[i] = sum;
            // Serial.print(dx[i]);
            // Serial.print(" ");
        }
        
        /*
        * the updated point in the chart is mapped to a quaternion
        */
        fC2M( dx , q_del );  // now delta is stored in q_del
#if GYRO_ONLY
        q = q_p;
#else
        q = q_p * q_del;
#endif
        q.normalize();
        // and the angular velocity is updated in the usual way
        w[0] += dx[3];
        w[1] += dx[4];
        w[2] += dx[5];
        // w = w_m.todouble();
        
        // the covariance matrix is updated in the chart centered in qp
        static double T[36];
        //S = -K*H
        for(int i=0; i<6; i++){
            for(int j=0; j<6; j++){
                double sum = 0.0;
                for(int k=0; k<9; k++) sum -= K[i*9+k]*H[k+j*9];
                T[i*6+j] = sum;
            }
        }
        // S = I-K*H
        for(int k=0; k<36; k+=7) T[k] += 1.0;
        // P_updated_in_q_predicted_centered = (I - K*H)*P_predicted
        for(int i=0; i<6; i++){
            for(int j=0; j<6; j++){
                double sum = 0.0;
                for(int k=0; k<6; k++) sum += T[i*6+k]*P[k+j*6];
                M[i+j*6] = sum;
            }
        }
        // Make symmetric
        for(int i=0; i<6; i++){
            for(int j=0; j<6; j++) 
                P[i+j*6] = 0.5*( M[i+j*6] + M[j+i*6] );
        }
        
        return;
    }

    void compute_K( double* S , double* M ){
        // we first compute the Cholesky decomposition for transform the system from  K*S = M  into K*L*L' = M
        Cholesky9( S );
        // for(int i =0; i<9; i++){
        //     Serial.print(S[i*10]);
        //     Serial.print(" ");
        // }
        double y[9];
        // then we take each pair of rows of K and M independently
        for(int i=0; i<6; i++){
            // first we solve (y*L' = M)
            for(int j=0; j<9; j++){
                double sum = 0.0;
                for(int k=0; k<j; k++){
                    sum += y[k]*S[j+k*9];
                }
                y[j] = ( M[i*9+j] - sum )/S[j*10];
            }
            // now we solve (Ki*L = y)
            for(int j=8; j>-1; j--){
                double sum = 0.0;
                for(int k=j+1; k<9; k++){
                    sum += M[i*9+k]*S[k+j*9];
                }
                M[i*9+j] = ( y[j] - sum )/S[j*10];
            }
        }
        for(int i=0; i<54; i++){
            K[i]=M[i];
            
            // Serial.print(K[i]);
            // Serial.print(" ");
        }
            
        return;
    }

    void Cholesky9( double* S ){
        // for each column
        for(int j=0; j<9; j++){
            double sum = 0.0;  //sum for the diagonal term
            // we first fill with 0.0 until diagonal
            for(int i=0; i<j; i++){
                S[i+j*9] = 0.0;
                //we can compute this sum at the same time
                sum += S[j+i*9]*S[j+i*9];
            }
            // now we compute the diagonal term

            S[j*10] = sqrt( S[j*10] - sum ); //S[j+j*m] = sqrt( S[j+j*m] - sum );

            // finally we compute the terms below the diagonal
            for(int i=j+1; i<9; i++){
                //first the sum
                sum = 0.0;
                for(char k=0; k<j; k++){
                    sum += S[i+k*9]*S[j+k*9];
                }
                //after the non-diagonal term
                S[i+j*9] = ( S[i+j*9] - sum )/S[j*10];
            }
        }//end j
        return;
    }

    // Method: fC2M
    // defines the map from the chart points, to the manifold points (through the delta quaternion)
    // inputs:
    //  e: point of the Euclidean space that we want to map to a unit quaternion
    // outputs:
    //  delta: quaternion mapped with the e point
    void fC2M( double* e , QuaternionD& delta ){
        // delta from the chart definition: Rotation Vector
        double enorm = sqrt( e[0]*e[0] + e[1]*e[1] + e[2]*e[2] );
        if( enorm > PI ){
            double aux = PI/enorm;
            e[0] *= aux;
            e[1] *= aux;
            e[2] *= aux;
            enorm = PI;
        }
        if( enorm != 0.0 ){
            double aux = sin(0.5*enorm)/enorm;
            delta[0] = cos(0.5*enorm);
            delta[1] = e[0]*aux;
            delta[2] = e[1]*aux;
            delta[3] = e[2]*aux;
        }else{
            delta[0] = 1.0;
            delta[1] = 0.0;
            delta[2] = 0.0;
            delta[3] = 0.0;
        }
        return;
    }

      // Method: chartUpdateMatrix
  // this function defines the transformation on the covariance matrix
  // when it is redefined from the chart centered in q quaternion, to the
  // chart centered in p quaternion, being them related by  p = q * delta
  // inputs:
  //  delta: quaternion used to update the quaternion estimation
  // outputs:
  //  G: transformation matrix to update the covariance matrix
    void chartUpdateMatrix( QuaternionD& delta , double* G ){
        double dnorm = sqrt( delta[1]*delta[1] + delta[2]*delta[2] + delta[3]*delta[3] );
        if( dnorm != 0.0 ){
            double udelta[3];
            udelta[0] = delta[1]/dnorm;
            udelta[1] = delta[2]/dnorm;
            udelta[2] = delta[3]/dnorm;
            double dnasindn = dnorm/asin(dnorm);
            // we will not use delta again in this update, so we transform it to save computations
            delta[0] *= dnasindn;
            delta[1] *= dnasindn;
            delta[2] *= dnasindn;
            delta[3] *= dnasindn;
            G[0] = delta[0];     G[3] = delta[3];     G[6] = -delta[2];
            G[1] = -delta[3];    G[4] = delta[0];     G[7] = delta[1];
            G[2] = delta[2];     G[5] = -delta[1];    G[8] = delta[0];
            for(int i=0; i<3; i++){
                for(int j=0; j<3; j++) G[i+j*3] += (1.0-delta[0])*udelta[i]*udelta[j];
            }
        }else{
            for(int k=0; k<9; k++) G[k] = 0.0;
            for(int k=0; k<9; k+=4) G[k] = 1.0;
        }
        
        return;
    }
};
