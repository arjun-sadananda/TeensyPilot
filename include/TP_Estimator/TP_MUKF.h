#pragma once

#include "quaternion.h"
#include <Arduino.h>
#include "Wire.h"
// #include "matrixN.h"
// #include "AP_Math.h"

// #define GYRO_ONLY true

// Thanks to: https://github.com/PBernalPolo/test_MKF.git

class TP_MUKF
{
private:
protected:
    // PRIVATE VARIABLES
    // quaternion describing the orientation (q1,q2,q3,q4)=(qx,qy,qz,qw) wrong order !
    // (rotation that transform vectors from the sensor reference frame, to the external reference frame)
    // double q[4];
    QuaternionD q;
    // angular velocity (rad/s)
    // double w[3];
    Vector3d w;
    // covariance matrix of the angular velocity noise (rad^2/s^3)
    double Qw;
    // covariance matrix of the acceleration noise (g^2)
    double Qa;
    // covariance matrix of the angular velocity measurement noise (rad^2/s^2)
    double Rw;
    // covariance matrix of the acceleration measurement noise (g^2)
    double Ra;
    // use or not the chart update
    boolean chartUpdate = true;
    double W0;

    // Quaternion quat;
    double q0[4];
    double e[3];

public:
    // covariance matrix
    double P[36];
    // double K[36];
    Vector3d v_ref;
    TP_MUKF(){
        q[0] = 1.0;   q[1] = 0.0;   q[2] = 0.0;   q[3] = 0.0;
        // it is necessary to set an angular velocity different from 0.0 to break the symmetry
        // otherwise, the MUKF could not converge (especially when we apply the "reset operation" with the RV chart)
        w[0] = 1.0e-10;   w[1] = 1.0e-10;   w[2] = 1.0e-10;
        
        for(int k=0; k<36; k++) P[k] = 0.0;
        for(int k=0; k<36; k+=7) P[k] = 1.0e2;
        P[2+2*6] = 1.0e-16;
        
        Qw = 1.0e1;
        Qa = 1.0e-2;
        Rw = 1.0e-3; // Gyro Measure Noise
        Ra = 1.0e-3;

        W0 = 1.0/25.0;

        
        q0[0] = 1.0;   q0[1] = 0.0;   q0[2] = 0.0;   q0[3] = 0.0;
        e[0] = 0.0;   e[1] = 0.0;   e[2] = 0.0;
    }

    Quaternion get_q(){
        static Quaternion ret;
        // ret[0] = (float) q[3];
        for(int i=0; i<4; i++) ret[i] = (float) q[i];
        return ret;
    }
    
    void set_chartUpdate( boolean chartUpdateIn ){
        chartUpdate = chartUpdateIn;
    }

    void init_estimator(const Vector3f v_r){
        // R[0] = R[1] = R[2] = sq(mpu.std_dev_accel);
        // R[3] = R[4] = R[5] = sq(mag.std_dev_mag);
        // sigma_gyro = mpu.std_dev_gyro;
        // q[0] = 1.0;
        // q[1] = 0.0;
        // q[2] = 0.0;
        // q[3] = 0.0;
        v_ref = v_r.todouble();
    }
     // Method: updateIMU
  // method used to update the state information through an IMU measurement
  // inputs:
  //  am: measured acceleration (g)
  //  wm: measured angular velocity (rad/s)
  //  dt: time step from the last update (s)
  // outputs:
    void estimate_attitude(Vector3f v, Vector3f gyro, const double dt){
        // ********************** Augmented Covariance Matrix *********************************
        double Pe[144];  // Pe is 12x12
        for(int k=0; k<144; k++)        Pe[k] = 0.0;            // 12, 24, 36, 48, 60, 72,... 84, 96, 108,... 120, 132, 144
        for(int i=0; i<6; i++) 
            for(int j=0; j<6; j++)      Pe[i+j*12] = P[i+j*6];  
        for(int i=0; i<3; i++)          Pe[78+i+i*12] = Qw;     // 72+6
        for(int i=0; i<3; i++)          Pe[117+i+i*12] = Qa;    // 108+9


        // ************************** Generate Sigma Points *************************************

        // we get the square-root of the matrix using the Cholesky factorization
        Cholesky( Pe , 12 );

        // Set Weights (W0 must be in [0,1])
        double Wi = (1.0-W0)/(2.0*12);

        //the factor for the square root, so P = sum(W_k*sigma_k)
        double alpha = 1.0/sqrt(2.0*Wi);
        for(int k=0; k<144; k++) 
            Pe[k] *= alpha;

        // we define and initialize the sigma points (state and measure)
        // 25 = 1 + 2*12  sigma points;  13 = 4+3+3 : quaternion, disturbance in w and a
        // 25 measurement predictions ;  6  = 3+3   : w_p and a_p 
        double X[25][13];
        double Y[25][6];
        GenerateSigmaPoints(X, Y, Pe);

        // ********************************* Prediction *****************************************

        // we compute the predictions
        for(int j=0; j<25; j++){
            statePrediction( X[j] , dt );
            // we make sure that all quaternions are in the same hemisphere
            double prod = 0.0;
            for(int i=0; i<4; i++) prod += X[0][i]*X[j][i];
                if( prod < 0.0 ){
                    for(int i=0; i<4; i++) X[j][i] = -X[j][i];
            }
            //if( X[j][0] < 0.0 ) for(int i=0; i<4; i++) X[j][i] = -X[j][i];  // this is an alternative

            IMU_MeasurementPrediction( Y[j] , X[j] , v_ref);
        }

        // ***************************************  Compute the Mean  *************************************************
        double xmean[7];
        double ymean[6];
        for(int i=0; i<7; i++) xmean[i] = W0*X[0][i];
        for(int i=0; i<6; i++) ymean[i] = W0*Y[0][i];
        for(int j=1; j<25; j++){
            for(int i=0; i<7; i++) xmean[i] += Wi*X[j][i];
            for(int i=0; i<6; i++) ymean[i] += Wi*Y[j][i];
        }
        double qmeanNorm = sqrt( xmean[0]*xmean[0] + xmean[1]*xmean[1] + xmean[2]*xmean[2] + xmean[3]*xmean[3] );
        for(int i=0; i<4; i++) xmean[i] /= qmeanNorm;

        // *******************************  Compute the covariance matrices  *******************************************
        double Pxx[6*6];
        double Pxy[6*6];
        double Pyy[6*6];
        double dX [6];
        double dY[6];
        //   first we add the 0 contribution
        fM2C( dX , xmean , X[0] );
        for(int i=3; i<6; i++) 
            dX[i] = X[0][i+1]-xmean[i+1];
        for(int i=0; i<6; i++) 
            dY[i] = Y[0][i]-ymean[i];
        for(int i=0; i<6; i++){
            for(int j=0; j<6; j++){
                Pxx[i+j*6] = W0*dX[i]*dX[j];
                Pxy[i*6+j] = W0*dX[i]*dY[j];
                Pyy[i+j*6] = W0*dY[i]*dY[j];
            }
        }
        //   then the rest
        for(int k=1; k<25; k++){
            fM2C( dX , xmean , X[k] );
            for(int i=3; i<6; i++) 
                dX[i] = X[k][i+1]-xmean[i+1];
            for(int i=0; i<6; i++) 
                dY[i] = Y[k][i]-ymean[i];
            for(int i=0; i<6; i++){
                for(int j=0; j<6; j++){
                    Pxx[i+j*6] += Wi*dX[i]*dX[j];
                    Pxy[i*6+j] += Wi*dX[i]*dY[j];
                    Pyy[i+j*6] += Wi*dY[i]*dY[j];
                }
            }
        }
        //   finally we add the noise (the linear part)
        for(int i=0; i<3; i++){                                                     /// Modified
            Pyy[i+i*6] += Ra;
            Pyy[i+3+(i+3)*6] += Rw;
        }

        // ******************************************* Update ***************************************************
        // we save Pyy in other matrix because solve() will overwrite it
        for(int k=0; k<36; k++) P[k] = Pyy[k];

        // now we can compute the gain ( K*Pyy = Pxy )
        solve(P , Pxy );  // now K is stored in Pxy

        // and update the state in the chart
        double dy[] = { v[0]-ymean[0] , v[1]-ymean[1] , v[2]-ymean[2] , gyro[0]-ymean[3] , gyro[1]-ymean[4] , gyro[2]-ymean[5] };

        double dx[6]; // dx = Kn*(zn-zp)
        for(int i=0; i<6; i++){
            double sum = 0.0;
            for(int j=0; j<6; j++) 
                sum += Pxy[i*6+j]*dy[j];
            dx[i] = sum;
        }

        // this update takes place in the chart centered in xmean
        for(int i=0; i<4; i++) q0[i] = xmean[i];
        for(int i=0; i<3; i++) e[i] = dx[i];

        // the updated point in the chart is mapped to a quaternion
        double temp[4];
        fC2M( temp , q0 , e );
        for(int i=0; i<4; i++) q[i]=temp[i];
        // and the angular velocity is updated in the usual way
        w[0] = xmean[4] + dx[3];
        w[1] = xmean[5] + dx[4];
        w[2] = xmean[6] + dx[5];

        // the covariance matrix is updated in the chart centered in q0 ( P = Pxx - K*Pyy*K^T )
        for(int i=0; i<6; i++){
            for(int j=0; j<6; j++){
            double sum = 0.0;
            for(int k=0; k<6; k++) sum += Pyy[i+k*6]*Pxy[j*6+k];
                P[i+j*6] = sum;
            }
        }
        for(int i=0; i<6; i++){
            for(int j=0; j<6; j++){
            double sum = 0.0;
            for(int k=0; k<6; k++) sum += Pxy[i*6+k]*P[k+j*6];
            Pxx[i+j*6] -= sum;
            }
        }

        // we avoid numerical instabilities
        q.normalize();
        for(int i=0; i<6; i++){
            for(int j=0; j<6; j++) 
                P[i+j*6] = 0.5*( Pxx[i+j*6] + Pxx[j+i*6] );
        }

        // this covariance matrix is expressed in the q0 chart
        // we will have to update it to the new q chart
        // that is why we do what we do at the begining  
        // Line ~147

        return;
    }

    void GenerateSigmaPoints( double X[25][13], double Y[25][6], double* Pe){
        for(int j=0; j<25; j++)
            for(int i=0; i<13; i++) X[j][i] = 0.0;
        for(int j=0; j<25; j++)
            for(int i=0; i<6; i++) Y[j][i] = 0.0;

        // first we set the mean value
        for(int i=0; i<4; i++) X[0][i] = q[i]; // quaternion! not chart point.... ????
        for(int i=0; i<3; i++) X[0][i+4] = w[i];
        for(int i=0; i<6; i++) X[0][i+7] = 0.0;

        //???? we can test if the Chart update is good or not by uncommenting the next lines
        if( !chartUpdate ){
            for(int i=0; i<4; i++) q0[i] = q[i];
            for(int i=0; i<3; i++) e[i] = 0.0;
        }

        // second we generate the +sigma points from the P matrix
        for(int j=0; j<12; j++){
            // we do this because P is expressed in the q0 chart, but we need to
            // express it in the q chart for the next time step
            //   first we compute the point in the chart
            double eP[] = { e[0]+Pe[0+j*12] , e[1]+Pe[1+j*12] , e[2]+Pe[2+j*12] };
            //   we get the point in the manifold
            fC2M( X[j+1] , q0 , eP );
            // we set the angular velocity
            for(int i=3; i<6; i++) X[j+1][i+1] = w[i-3] + Pe[i+j*12];
            for(int i=6; i<12; i++) X[j+1][i+1] = Pe[i+j*12];
        }
        // third we generate the -sigma points from the P matrix
        for(int j=0; j<12; j++){
            // we do this because P is expressed in the q0 chart, but we need to
            // express it in the q chart for the next time step
            //   first we compute the point in the chart
            double eP[] = { e[0]-Pe[0+j*12] , e[1]-Pe[1+j*12] , e[2]-Pe[2+j*12] };
            //   we get the point in the manifold
            fC2M( X[j+13] , q0 , eP );
            // we set the angular velocity
            for(int i=3; i<6; i++) X[j+13][i+1] = w[i-3] - Pe[i+j*12];
            for(int i=6; i<12; i++) X[j+13][i+1] = -Pe[i+j*12];
        }
    }

    // Method: Cholesky
    // performs the Cholesky decomposition of a positive definite matrix ( S = L*L' )
    // inputs:
    //  S: positive definite matrix to be decomposed (must be stored by columns)
    //  n: number of rows or column of the square matrix S (nxn)
    // outputs:
    //  S: the lower triangular matrix L (nxn) is overwritten in S (is stored by columns)
    void Cholesky( double* S , int n ){
        // for each column
        for(int j=0; j<n; j++){
            double sum = 0.0;  //sum for the diagonal term
            // we first fill with 0.0 until diagonal
            for(int i=0; i<j; i++){
                S[i+j*n] = 0.0;
                //we can compute this sum at the same time
                sum += S[j+i*n]*S[j+i*n];
        }
        // now we compute the diagonal term
        S[j*(n+1)] = sqrt( S[j*(n+1)] - sum ); //S[j+j*m] = sqrt( S[j+j*m] - sum );
        // finally we compute the terms below the diagonal
        for(int i=j+1; i<n; i++){
            //first the sum
            sum = 0.0;
            for(char k=0; k<j; k++)
                sum += S[i+k*n]*S[j+k*n];
            //after the non-diagonal term
            S[i+j*n] = ( S[i+j*n] - sum )/S[j*(n+1)];
        }
        }//end j

        return;
    }

    // Method: fC2M
    // defines the map from the chart points, to the manifold points
    // inputs:
    //  qm: mean quaternion of the distribution (it is mapped with the origin of the chart)
    //  e: point of the chart that we want to map to a unit quaternion in the manifold
    // outputs:
    //  q: quaternion in the manifold mapped with the e point in the chart
    void fC2M( double* q , double* qm , double* e ){
        // delta from the chart definition: Modified Rodrigues Parameters
        double enorm = sqrt( e[0]*e[0] + e[1]*e[1] + e[2]*e[2] );
        if( enorm > 4.0 ){
            double aux = 4.0/enorm;
            e[0] = e[0]*aux;
            e[1] = e[1]*aux;
            e[2] = e[2]*aux;
            enorm = 4.0;
        }
        double aux = 1.0/( 16.0 + enorm*enorm );
        double delta[4];
        delta[0] = ( 16.0 - enorm*enorm )*aux;
        delta[1] = 8.0*e[0]*aux;
        delta[2] = 8.0*e[1]*aux;
        delta[3] = 8.0*e[2]*aux;
        // now we update in the manifold with this delta
        q[0] = qm[0]*delta[0] - qm[1]*delta[1] - qm[2]*delta[2] - qm[3]*delta[3];
        q[1] = qm[0]*delta[1]  +  delta[0]*qm[1]  +  qm[2]*delta[3] - qm[3]*delta[2];
        q[2] = qm[0]*delta[2]  +  delta[0]*qm[2]  +  qm[3]*delta[1] - qm[1]*delta[3];
        q[3] = qm[0]*delta[3]  +  delta[0]*qm[3]  +  qm[1]*delta[2] - qm[2]*delta[1];

        return;
    }

    // Method: fM2C
    // defines the map from the manifold points, to the chart points
    // inputs:
    //  qm: mean quaternion of the distribution (is mapped with the origin of the chart)
    //  q: quaternion that we want to map with a point in the chart
    // outputs:
    //  e: point in the chart mapped with the q quaternion
    void fM2C( double* e , double* qm , double* q ){
        // first we compute the delta in the manifold
        double delta[4];
        delta[0] = qm[0]*q[0] + qm[1]*q[1] + qm[2]*q[2] + qm[3]*q[3];
        delta[1] = qm[0]*q[1]  -  q[0]*qm[1]  -  qm[2]*q[3] + qm[3]*q[2];
        delta[2] = qm[0]*q[2]  -  q[0]*qm[2]  -  qm[3]*q[1] + qm[1]*q[3];
        delta[3] = qm[0]*q[3]  -  q[0]*qm[3]  -  qm[1]*q[2] + qm[2]*q[1];
        // e from the chart definition: Modified Rodrigues Parameters
        if( delta[0] < 0.0 ) for(int i=0; i<4; i++) 
            delta[i] = -delta[i];
        double aux = 4.0/( 1.0 + delta[0] );
        e[0] = delta[1]*aux;
        e[1] = delta[2]*aux;
        e[2] = delta[3]*aux;

        return;
    }

    // Method: statePrediction
    // this method predicts the state given the previous state, and the time increment
    // inputs:
    //  x: previous state (q,w,n,a)
    //  dt: time step
    // outputs:
    //  xp: predicted state (qp,wp,np,ap)
    void statePrediction( double* x , double dt ){
        // first we predict the angular velocity
        double wp[] = { x[4]+x[7]*dt , x[5]+x[8]*dt , x[6]+x[9]*dt };
        // angular velocity norm computation
        double wnorm = sqrt( wp[0]*wp[0] + wp[1]*wp[1] + wp[2]*wp[2] );
        // we compute qw
        double qw[4];
        if( wnorm != 0.0 ){
            double wdt05 = 0.5*wnorm*dt;
            double swdt = sin(wdt05)/wnorm;
            qw[0] = cos(wdt05);
            qw[1] = wp[0]*swdt;
            qw[2] = wp[1]*swdt;
            qw[3] = wp[2]*swdt;
        }else{
            qw[0] = 1.0;
            qw[1] = 0.0;
            qw[2] = 0.0;
            qw[3] = 0.0;
        }
        // we compute the predicted state (q*qw,w)
        double qp[4];
        qp[0] = x[0]*qw[0] - x[1]*qw[1] - x[2]*qw[2] - x[3]*qw[3];
        qp[1] = x[0]*qw[1]  +  qw[0]*x[1]  +  x[2]*qw[3] - x[3]*qw[2];
        qp[2] = x[0]*qw[2]  +  qw[0]*x[2]  +  x[3]*qw[1] - x[1]*qw[3];
        qp[3] = x[0]*qw[3]  +  qw[0]*x[3]  +  x[1]*qw[2] - x[2]*qw[1];
        // we overwrite the predicted state in the current state
        for(int i=0; i<4; i++) x[i] = qp[i];
        for(int i=0; i<3; i++) x[i+4] = wp[i];

        return;
    }
  
    // Function: IMU_MeasurementPrediction
    // this method predicts the measurement given a state
    // inputs:
    //  xp: state for which the measure is to be predicted
    // outputs:
    //  yp: predicted measurement
    void IMU_MeasurementPrediction( double* y , double* x , Vector3d v_ref){
        // the predicted acceleration measurement will be the gravity vector measured
        // in the sensor frame: g = (R^T)*[a-(0,0,-1)]
        //  first we compute the rotation matrix

        // double RT[9];
        // RT[0] = -x[2]*x[2]-x[3]*x[3];    RT[3] = x[1]*x[2]+x[3]*x[0];     RT[6] = x[1]*x[3]-x[2]*x[0];
        // RT[1] = x[1]*x[2]-x[3]*x[0];     RT[4] = -x[1]*x[1]-x[3]*x[3];    RT[7] = x[2]*x[3]+x[1]*x[0];
        // RT[2] = x[1]*x[3]+x[2]*x[0];     RT[5] = x[2]*x[3]-x[1]*x[0];     RT[8] = -x[1]*x[1]-x[2]*x[2];

        // RT[0] += RT[0] + 1.0;    RT[3] += RT[3];          RT[6] += RT[6];
        // RT[1] += RT[1];          RT[4] += RT[4] + 1.0;    RT[7] += RT[7];
        // RT[2] += RT[2];          RT[5] += RT[5];          RT[8] += RT[8] + 1.0;

        // double ag[] = { x[10] + v_ref[0], x[11] + v_ref[1], x[12] + v_ref[2] };
        // for(int i=0; i<3; i++){
        //     double sum = 0.0;
        //     for(int j=0; j<3; j++) sum += RT[i+j*3]*ag[j];
        //     y[i] = sum;
        // }

        static QuaternionD q;
        for(int i=0; i<4; i++) q[i] = x[i];

        static Vector3d ag;
        ag.set(x[10], x[11], x[12]);
        ag += v_ref;
        q.inverse().earth_to_body(ag);

        y[0] = ag[0];
        y[1] = ag[1];
        y[2] = ag[2];
        // the predicted measurement for the angular velocity will be itself
        y[3] = x[4];
        y[4] = x[5];
        y[5] = x[6];

        return;
    }

    // Method: solve
    // solves the system of linear equations  K*S = M  for K
    // inputs:
    //  S: 6x6 positive definite matrix stored by columns
    //  M: 6x6 matrix stored by rows
    // outputs:
    //  M: K (6x6) is stored by rows in the M memory space
    void solve( double* S , double* M ){
        // we first compute the Cholesky decomposition for transform the system from  K*S = M  into K*L*L' = M
        Cholesky( S , 6 );

        double y[6];
        // then we take each pair of rows of K and M independently
        for(int i=0; i<6; i++){
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

        return;
    }

};
