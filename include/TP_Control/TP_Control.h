
// #include<vector3.h>
// #include<AP_Math.h>

#include "TP_Motors\TP_Motors.h"


Vector3f SO3_attitude_control(const Matrix3f R, const Vector3f omega){

    static Vector3f e_R, e_Omega, M;
    static float k_R = 4, k_Omega = .01;
    static Matrix3f J;
    J.identity();

    Matrix3f R_err = R-R.transposed();                  // This is skew symmetric
    e_R.set(R_err.c.y/2, R_err.a.z/2, R_err.b.x/2);     // Can test omega control and angle control independently
                                                        // e_R      \in (-1, 1)^3
    e_Omega = omega;                                    // e_Omega  \in  rad/sec

    // ***
    // To tackle magnetic disturbance affecting yaw...
    // How about different k for 3 components???
    M = -e_R*k_R -e_Omega*k_Omega;// + omega%(J*omega);

    return M;
}

Vector3f euler_attitude_control(const Matrix3f R, const Vector3f omega){
    
    static Vector3f e_R, e_Omega, M;
    // static float k_R = 4, k_Omega = 1;
    static Matrix3f J;
    J.identity();

    e_R = R.to_euler312();              //                      // e_R \in (-1.5, 1.5)
    e_Omega = omega;

    M = - e_R;// * k_R - e_Omega * k_Omega;// + omega%(J*omega);   // M must be in Nm

    return M;
}

const bool ignore_yaw = true;

void command_motors(const float f, const Vector3f M, int *vel){
    static float f1, f2, f3, f4;
    // f = 1
    static float d = .142, c_inv = 1; // d 142mm .142 m, c in units
    // Props In configuration

    if (ignore_yaw)
        c_inv = 0;

    f1 = (f - M.x/d + M.y/d - M.z*c_inv )/4;
    f2 = (f - M.x/d - M.y/d + M.z*c_inv )/4;
    f3 = (f + M.x/d + M.y/d + M.z*c_inv )/4;
    f4 = (f + M.x/d - M.y/d - M.z*c_inv )/4;

    vel[0] = vel[1] = vel[2] = vel[3] = 1000;

    // This is not good
    // Maybe find max f
    // scale everything to bring max f within range?
    // Needs more thought
    if (f1>1) f1 = 1;
    if (f2>1) f2 = 1;
    if (f3>1) f3 = 1;
    if (f4>1) f4 = 1;

    if (f1<0) f1 = 0;
    if (f2<0) f2 = 0;
    if (f3<0) f3 = 0;
    if (f4<0) f4 = 0;

    vel[0] = 1050 + f1*200;
    vel[1] = 1050 + f2*200;
    vel[2] = 1050 + f3*200;
    vel[3] = 1050 + f4*200;

    set_motor_speeds(vel[0], vel[1], vel[2], vel[3]);
}