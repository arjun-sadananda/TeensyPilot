
// #include<vector3.h>
// #include<AP_Math.h>

#include "TP_Motors\TP_Motors.h"

#define MOTOR_ON true

class TP_Control : protected TP_MOTOR{
public:
    Vector3f M;
    float f;
    int v[4];
    // DO NOT ignore Yaw, It causes problem in roll and pitch as well!!
    const bool ignore_yaw = false;
    
    int get_max_vel(){
        return int(f_cap*hover_throttle);
    }
    float get_f_cap(){
        return f_cap;
    }
private:
    Vector3f e_R, e_Omega, e_R_I;
    const float k_R = 1, k_Omega = 0.11, k_I = 0.002;//.01;

    Matrix3f R_err;

    float vertical_rel_thrust = 1; // redundant
    const float f_cap = 1.5, f_arm = 0.1; //arm_throttle = 1050;
    int hover_throttle = 280; // arm_throttle = 1000 + hover_throttle * f_arm = 1020 
    const int hover_offset_2 = 0, hover_offset_4 = 0; // -10  0
    const int hover_offset_1 = 0, hover_offset_3 = 0; // -80 -70


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

    Vector3f SO3_attitude_control(const Matrix3f R, const Vector3f omega){
        static Vector3f M;
        static Matrix3f J;
        J.identity();

        R_err = R-R.transposed();                  // This is skew symmetric
        e_R.set(R_err.c.y/2, R_err.a.z/2, R_err.b.x/2);     // Can test omega control and angle control independently
                                                            // e_R      \in (-1, 1)^3
        e_Omega = omega;                                    // e_Omega  \in  rad/sec

        static float phi;
        phi = 3 - (R.a.x + R.b.y + R.c.z);
        if (phi < .3)
            e_R_I += e_R;
        // if (e_R_I.length())
        // ***
        // To tackle magnetic disturbance affecting yaw...
        // How about different k for 3 components???
        M = -e_R*k_R -e_Omega*k_Omega - e_R_I*k_I;// + omega%(J*omega);

        return M;
    }

    void get_motor_commands(){
        static float f1, f2, f3, f4;
        // f = 1
        static float c_inv = 1.2, k_d = .8;// d = 1,  d<1 -> more sensitive to roll commands // d 142mm .142 m, c in units
        // Props In configuration

        if (ignore_yaw)
            c_inv = 0;

        // f1 = (f - M.x/d + M.y/d + M.z*c_inv )/4;
        // f2 = (f - M.x/d - M.y/d - M.z*c_inv )/4;
        // f3 = (f + M.x/d + M.y/d - M.z*c_inv )/4;
        // f4 = (f + M.x/d - M.y/d + M.z*c_inv )/4;

        f1 = f - k_d*M.x + k_d*M.y + M.z*c_inv;
        f2 = f - k_d*M.x - k_d*M.y - M.z*c_inv;
        f3 = f + k_d*M.x + k_d*M.y - M.z*c_inv;
        f4 = f + k_d*M.x - k_d*M.y + M.z*c_inv;

        // f1 = f - M.x + M.y + M.z*c_inv; // yaw was ulta
        // f2 = f - M.x - M.y - M.z*c_inv;
        // f3 = f + M.x + M.y - M.z*c_inv;
        // f4 = f + M.x - M.y + M.z*c_inv;

        v[0] = v[1] = v[2] = v[3] = 1000;

        // This is not good
        // Maybe find max f
        // scale everything to bring max f within range?
        // Needs more thought
        // if (f1<0) {f1 =  0;     f2 -= f1;      f3 -= f1;     f4 -= f1;}
        // if (f2<0) {f1 -= f2;    f2  = 0;       f3 -= f2;     f4 -= f2;}
        // if (f3<0) {f1 -= f3;    f2 -= f3;      f3  = 0;      f4 -= f3;}
        // if (f4<0) {f1 -= f4;    f2 -= f4;      f3 -= f3;     f4  = 0; }

        // if (f1>1) {f1  = 1;     f2 /= f1;     f3 /= f1;     f4 /= f1;}
        // if (f2>1) {f1 /= f2;    f2  = 1;      f3 /= f2;     f4 /= f2;}
        // if (f3>1) {f1 /= f3;    f2 /= f3;     f3  = 1;      f4 /= f3;}
        // if (f4>1) {f1 /= f4;    f2 /= f4;     f3 /= f4;     f4  =  1;}

        
        // Check/Fix this
        if (f1<f_arm) f1 = f_arm;
        if (f2<f_arm) f2 = f_arm;
        if (f3<f_arm) f3 = f_arm;
        if (f4<f_arm) f4 = f_arm;

        if (f1>f_cap) f1 = f_cap;
        if (f2>f_cap) f2 = f_cap;
        if (f3>f_cap) f3 = f_cap;
        if (f4>f_cap) f4 = f_cap;

        // f1 = f2 = f3 = f4 = 1; //For testing open loop speed control

        v[0] = 1000 + f1 * (hover_throttle + hover_offset_1);
        v[1] = 1000 + f2 * (hover_throttle + hover_offset_2);
        v[2] = 1000 + f3 * (hover_throttle + hover_offset_3);
        v[3] = 1000 + f4 * (hover_throttle + hover_offset_4);
    }

public:

    void clear_integral(){
        e_R_I.zero();
    }
    void init_motors(){
        ESC_setup();
    }
    void throw_mode_controller(Quaternion q, Vector3f omega, bool motor_on){
        static Matrix3f R;
        // static const float m = 1, g=1; // 550gms
        q.rotation_matrix(R);
        // f = m*g*R.c.z;  // or 
        f = vertical_rel_thrust*R.c.z; // relative-force = f/mg   ---  f = 1 is hover force
        M = SO3_attitude_control( R, omega);
        get_motor_commands();
#if MOTOR_ON
        if (motor_on)
            set_motor_speeds(v[0], v[1], v[2], v[3]);
#endif
    }
    void stop_motors(){
        set_motor_speeds(1000, 1000, 1000, 1000);
    }
    void motor_test(){
        for(int i = 0; i<=300; i+=20){
            set_motor_speeds(1000+i, 1000+i, 1000+i, 1000+i);
            delay(3000);
        }
        stop_motors();
        delay(5000);
    }
};