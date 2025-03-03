
// #include<vector3.h>
// #include<AP_Math.h>

// #include "TP_Motor\TP_Motor_2DOF.h"
#include "TP_Motor\TP_Motor_2DOF_simple.h"

#define MOTOR_ON true

class TP_Control_2DOF : protected TP_MOTOR_2DOF_SIMPLE{
public:
    // static const float m = 1, g=1; // 550gms
    Vector3f M;
    float f;
    float v[4];
    // DO NOT ignore Yaw, It causes problem in roll and pitch as well!!
    const bool ignore_yaw = false;
    
private:

public:

    void init_motors(){
#if MOTOR_ON
        ESC_setup();
        stop_motors();
#endif
    }

    void twoDOF_go_to(float p, float y, float p_d, float y_d, bool motor_on){
        // pitch .8 to -.8
        // yaw -3.14 to 3.14
        static float m_pitch,m_yaw;
        static float e_p, e_y;
        
        // Quanser:
        //      18.9     1.98     7.48       1.53      7.03      0.77
        //      -2.22    19.4     -0.45      11.9      -0.77     7.03
        // x =  p        y        pd         yd        pi        yi
        float P_avg = 10;
        float Y_avg = 5;
        static float dt, e_pd, e_yd;
        static float e_p_prev, e_y_prev, t_prev;
        static float kp_pp = 8 , kp_py = -2, kd_pp =0,               kI_p = 1;
        static float kp_yp = 0 , kp_yy = 8 ,            kd_yy = 1;
        //1.0/.8*10.0
        //2.0/3.14*5.0
        static float mp_bias = 8.0;
        static float p_I = 0;

        // TODO Fix this difference
        e_p = p - p_d; //rad
        e_y = y - y_d; // rad
        dt = (micros()-t_prev)/1000000; // sec
        e_pd = (e_p - e_p_prev)/dt;     // rad/sec
        e_yd = (e_y - e_y_prev)/dt;     // rad/sec
        e_p_prev = e_p;
        e_y_prev = e_y;
        t_prev   = micros();
        if (e_p < .4 && e_p>-.4)
            p_I += e_p*dt;              // rad.sec
        

        m_pitch = kp_pp*e_p + kp_py*e_y + kd_pp*e_pd + kI_p*p_I + mp_bias;
        m_yaw =   kp_yp*e_p + kp_yy*e_y + kd_yy*e_yd;

#if MOTOR_ON
        if (motor_on)
            set_motor_speeds(m_yaw ,m_pitch);
#endif
    }

    
    void twoDOF_go_to_simple(float p, float y, float p_d, float y_d, bool motor_on){
        // pitch .8 to -.8
        // yaw -3.14 to 3.14
        static float m_pitch,m_yaw;
        static float e_p, e_y, e_y_smooth, e_p_smooth;
        
        // Quanser:
        //      18.9     1.98     7.48       1.53      7.03      0.77
        //      -2.22    19.4     -0.45      11.9      -0.77     7.03
        // x =  p        y        pd         yd        pi        yi
        static float dt, e_pd, e_yd;
        static float e_p_prev, e_y_prev, t_prev;
        const int N = 500;
        static float e_p_window[N], e_y_window[N], sum_p,sum_y;
        static float kp_pp = 20 , kd_pp = 4000, kI_p = 5;
        static float kp_yy = 15, Kp_yy = 15 , kd_yy = 10000; // might need kI_y

        static float mp_bias = 25.0;
        static float p_I = 0;
        
        // TODO Fix this difference
        e_p = p - p_d; //rad
        e_y = y_d - y; // rad
        //1.0/.8*10.0
        //2.0/3.14*5.0
        sum_y = 0;
        sum_p = 0;
        for (int i = 0; i<N-1; i++){
            e_y_window[i] = e_y_window[i+1];
            e_p_window[i] = e_p_window[i+1];
            sum_y += e_y_window[i];
            sum_p += e_p_window[i];
        }
        e_y_window[N-1] = e_y;
        e_p_window[N-1] = e_p;

        e_y_smooth = (sum_y+e_y)/N;
        e_p_smooth = (sum_p+e_p)/N;

        dt = (micros()-t_prev)/1000000; // sec
        e_pd = (e_p_smooth - e_p_prev);     // rad/sec
        e_yd = (e_y_smooth - e_y_prev);     // rad/sec
        e_p_prev = e_p_smooth;
        e_y_prev = e_y_smooth;
        t_prev   = micros();
        if (e_p < .4 && e_p>-.4)
            p_I += e_p*dt;              // rad.sec
        
        // float
        if (e_y_smooth < -1)
            kp_yy = .5*Kp_yy;
        else if (e_y_smooth <1)
            kp_yy = Kp_yy;
        else
            kp_yy = .5*Kp_yy;

        m_pitch = mp_bias + kp_pp*e_p + kd_pp*e_pd + kI_p*p_I;
        m_yaw   =           kp_yy*e_y + kd_yy*e_yd;

#if MOTOR_ON
        if (motor_on)
            set_motor_speeds(m_yaw ,m_pitch);
#endif
    }

    void stop_motors(){
#if MOTOR_ON
        set_motor_speeds(0, 0);
#endif
    }

    void motor_test(){
#if MOTOR_ON
        // for(float i = 0; i<=30000; i++){
        //     set_motor_speeds(i/1000.0, i/1000.0);
        //     delayMicroseconds(100);
        // }
        // for(float i = 0; i<=10; i+=10){
        //     set_motor_speeds(i, 0);
        //     delayMicroseconds(5000000);
        // }
        // for(float i = 0; i<=10; i+=10){
        //     set_motor_speeds(0, i);
        //     delayMicroseconds(5000000);
        // }
        // set_motor_speeds(5, 5);
        set_motor_speeds(0, 20);
        delay(3000);
        stop_motors();
        delay(3000);
        set_motor_speeds(20, 0);
        delay(3000);
        stop_motors();
        delay(3000);
#endif
    }

    void motor_test_3D(){
#if MOTOR_ON
        // for(float i = 0; i<=30000; i++){
        //     set_motor_speeds(i/1000.0, i/1000.0);
        //     delayMicroseconds(100);
        // }
        // for(float i = 0; i<=10; i+=10){
        //     set_motor_speeds(i, 0);
        //     delayMicroseconds(5000000);
        // }
        // for(float i = 0; i<=10; i+=10){
        //     set_motor_speeds(0, i);
        //     delayMicroseconds(5000000);
        // }
        // set_motor_speeds(5, 5);
        set_motor_speeds(0, 20);    delay(3000);
        stop_motors();              delay(1500);
        set_motor_speeds(20, 0);    delay(3000);
        stop_motors();              delay(1500);
        set_motor_speeds(0, -20);   delay(3000);
        stop_motors();              delay(1500);
        set_motor_speeds(-20, 0);   delay(3000);
        stop_motors();              delay(1500);
#endif
    }

    void run_all_motors(float throttle_percent){
#if MOTOR_ON
        set_motor_speeds(throttle_percent, throttle_percent);
#endif
        return;
    }
};