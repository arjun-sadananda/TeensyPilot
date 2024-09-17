
// #include<vector3.h>
// #include<AP_Math.h>

#include "TP_Motor\TP_Motor.h"

#define MOTOR_ON true

class TP_Control : protected TP_MOTOR{
public:
    // static const float m = 1, g=1; // 550gms
    Vector3f M;
    float f;
    float v[4];
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
    const float k_R = .75, k_Omega = 0.1, k_I = 0.002;//.01;

    Matrix3f R_err;

    float vertical_rel_thrust = 1; // redundant
    const float f_cap = 3.0, f_arm = 0.1; //arm_throttle = 1050;
    float hover_throttle = 30.0; // throttle when f=1; arm_throttle = 1000 + hover_throttle * f_arm = 1020 

    // uint16_t half_throttle = 300;
    const float hover_offset_2 = 10, hover_offset_4 = 10; // -10  0
    const float hover_offset_1 = 0, hover_offset_3 = 0; // -80 -70


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

        R_err = R-R.transposed();                           // This is skew symmetric
        e_R.set(R_err.c.y/2, R_err.a.z/2, R_err.b.x/2);     // Can test omega control and angle control independently
                                                            // e_R      \in (-1, 1)^3
        e_Omega = omega;                                    // e_Omega  \in  rad/sec

        static float phi;
        phi = 3 - (R.a.x + R.b.y + R.c.z);
        if (phi < .3)
            e_R_I += e_R;
        M = -e_R*k_R -e_Omega*k_Omega - e_R_I*k_I;          // + omega%(J*omega);
        // if (e_R_I.length())
        // ***
        // To tackle magnetic disturbance affecting yaw...
        // How about different k for 3 components???

        return M;
    }

    Vector3f SO3_attitude_control(const Matrix3f R, const Vector3f omega, const Matrix3f Rd){
        static Vector3f M;
        static Matrix3f J;
        J.identity();

        R_err = Rd.transposed()*R-R.transposed()*Rd;                  // This is skew symmetric
        e_R.set(R_err.c.y/2, R_err.a.z/2, R_err.b.x/2);     // Can test omega control and angle control independently
                                                            // e_R      \in (-1, 1)^3
        e_Omega = omega;                                    // e_Omega  \in  rad/sec

        // static float phi;
        // phi = 3 - (R.a.x + R.b.y + R.c.z);
        // if (phi < .3)
        //     e_R_I += e_R;
        // if (e_R_I.length()>300.0){
        //     e_R_I.normalize();
        //     e_R_I*=300.0;
        // }
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

        v[0] = v[1] = v[2] = v[3] = 0;

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

        v[0] = f1 * (hover_throttle) + hover_offset_1;
        v[1] = f2 * (hover_throttle) + hover_offset_2;
        v[2] = f3 * (hover_throttle) + hover_offset_3;
        v[3] = f4 * (hover_throttle) + hover_offset_4;

        // v[0] = 1000 + f1 * hover_throttle;
        // v[1] = 1000 + f2 * hover_throttle;
        // v[2] = 1000 + f3 * hover_throttle;
        // v[3] = 1000 + f4 * hover_throttle;
    }

public:

    void clear_integral(){
        e_R_I.zero();
    }

    void init_motors(){
#if MOTOR_ON
        ESC_setup();
        stop_motors();
#endif
    }

    void angle_mode_controller( const Quaternion q, 
                                const Vector3f omega, 
                                const float thrust, 
                                const Matrix3f Rd, 
                                bool motor_on){
        static Matrix3f R;
        q.rotation_matrix(R);
        f =  thrust *R.c.z;                             // relative-force = f/mg   ---  f = 1 is hover force         // f = m*g*R.c.z;  // or 
        // f =  thrust /R.c.z;                             // relative-force = f/mg   ---  f = 1 is hover force         // f = m*g*R.c.z;  // or 
        M = SO3_attitude_control( R, omega, Rd);
        get_motor_commands();
#if MOTOR_ON
        if (motor_on)
            set_motor_speeds(v[0], v[1], v[2], v[3]);
#endif
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
#if MOTOR_ON
        set_motor_speeds(0, 0, 0, 0);
#endif
    }

    void motor_test(){
#if MOTOR_ON
        for(float i = 0; i<=30000; i++){
            set_motor_speeds(i/1000.0, i/1000.0, i/1000.0, i/1000.0);
            delayMicroseconds(100);
        }
        stop_motors();
        delay(1000);
#endif
    }

    void run_all_motors(float throttle_percent){
#if MOTOR_ON
        set_motor_speeds(throttle_percent, throttle_percent, throttle_percent, throttle_percent);
#endif
        return;
    }
};