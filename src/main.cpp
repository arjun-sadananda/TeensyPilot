// #include <zephyr.h>
#include <Arduino.h>

// void main(void)
// {
// }

// int main(void)
// {
//     init();

// #if defined(USBCON)
//     USBDevice.attach();
// #endif
    
//     setup();
    
//     for (;;) {
//         loop();
//         if (serialEventRun) serialEventRun();
//     }
        
//     return 0;
// }

// #include "TP_vector3.h"
// #include "TP_matrix3.h"
// #include "TP_vectorN.h"
// #include "TP_matrixN.h"
// #include "TP_MPU6050.h"
// #include "TP_Mag5883.h"
// #include "TP_BMP280.h"
#include "AP_Math.h"
#include "TP_StateEstimate\TP_BKF.h"
#include "TP_StateEstimate\TP_TRIAD.h"
#include "TP_StateEstimate\TP_EKF.h"
#include "TP_StateEstimate\TP_MEKF.h"
#include "TP_Display\TP_Display.h"
#include "Wire.h"

// #define DEBUG_MODE true
// #define DEBUG_AHRS true
// #define DEBUG_ALT_EST true

#define BKF_DIPLAY true

#define BKF 0
#define TRIAD 1 
#define EKF 2
#define MEKF 3

#define TP_ESTIMATOR MEKF

#define NO_DISPLAY 0
#define ACC_MAG_BALL 1
#define BKF_DISPLAY 2
#define TRIAD_DISPLAY 3
#define EKF_DISPLAY 4
#define MEKF_DISPLAY 5

#define DISPLAY_MODE MEKF_DISPLAY

#define SERIAL_ON false

#if SERIAL_ON
    #define debug(x) Serial.print(x); Serial.print(" ");
    #define debugln(x) Serial.println(x)
#else
    #define debug(x) 
    #define debugln(x)
#endif

uint32_t loop_timer;

#define Tus 20000

TP_BKF tp_bkf;
TP_TRIAD tp_triad;
TP_EKF tp_ekf;
TP_MEKF tp_mekf;

TP_Display tp_display;
/*
main
*/
void setup() {

    // Serial.println("Starting Wire");
    Wire.setClock(400000);
    Wire.begin();
    delay(250);
    
    // pinMode(13, OUTPUT);
    // digitalWrite(13, LOW);

#if SERIAL_ON
    Serial.begin(57600);
#endif

#if DISPLAY_MODE != 0
    tp_display.display_setup(DISPLAY_MODE);
    tp_display.printStatus("Calibrating Gyro");
#endif
    debugln("Init and Calib Gyro");
    
#if TP_ESTIMATOR == BKF
    tp_bkf.init_sensors();
    tp_bkf.mpu.calibrate_gyro();
#elif TP_ESTIMATOR ==TRIAD
    tp_triad.init_sensors();
#elif TP_ESTIMATOR == EKF
    tp_ekf.init_sensors();
    tp_ekf.init_estimator();
    tp_ekf.mpu.calibrate_gyro();
    tp_ekf.mag.set_m_ref();
#elif TP_ESTIMATOR == MEKF
    tp_mekf.init_sensors();
    tp_mekf.init_estimator();
    tp_mekf.mpu.calibrate_gyro();
    // tp_mekf.mag.set_m_ref();
#endif


    // debugln("Calibrating Baro");
    // tp_display.printStatus("Calibrating Baro");
    // tp_bkf.baro.calibrate_baro();
    // Serial.println(BaroAltOffset);
    // Serial.println("Baro Calibration Done");
    tp_display.printStatus("Estimator Running");
    // digitalWrite(13, HIGH);
}

void loop() {
    loop_timer = micros();

#if TP_ESTIMATOR == BKF
    tp_bkf.attitude_estimate();
    tp_bkf.vert_state_estimate();
#elif TP_ESTIMATOR == TRIAD
    tp_triad.estimate();
#elif TP_ESTIMATOR == EKF
    tp_ekf.attitude_estimate();
#elif TP_ESTIMATOR == MEKF
    tp_mekf.estimate_attitude();
#endif

    // ------------------- For Serial Plotter -------------------------

#if DISPLAY_MODE == BKF_DISPLAY
    tp_display.draw_euler_deg(tp_bkf.attitude_euler.x, tp_bkf.attitude_euler.y, tp_bkf.attitude_euler.z);
    // rot.x = tp_bkf.attitude_euler.y;
    // rot.y = tp_bkf.attitude_euler.x;
    // rot.z = tp_bkf.attitude_euler.z;
    // rot.x = tp_bkf.accel_roll;
    // rot.y = tp_bkf.accel_pitch;
    // rot.z = 0;//tp_bkf.mag_yaw;
    // Matrix3f rot;
    // q.from_euler(rot);//*AP_DEG_TO_RAD);
    // a = tp_bkf.mpu.AccelBody;
    // m = tp_bkf.mag.UnitMagVect;
    tp_display.drawCube(tp_bkf.accel_roll, tp_bkf.accel_pitch, 0);
    debug(tp_bkf.accel_roll);
    debug(tp_bkf.accel_pitch);
    debugln(tp_bkf.mag_yaw);
#elif DISPLAY_MODE == ACC_MAG_BALL
    tp_display.draw_acc_mag_in_ball(tp_triad.mpu.UnitAccelBody, tp_triad.mag.UnitMagVect);
    tp_display.printVector(tp_triad.mpu.UnitAccelBody, 280, 50);
    tp_display.printVector(tp_triad.mag.UnitMagVect, 280, 100);
#elif DISPLAY_MODE == TRIAD_DISPLAY
    tp_display.draw_acc_mag_in_ball(tp_triad.mpu.UnitAccelBody, tp_triad.mag.UnitMagVect);
    // Transposed to display cude as fixed and display moving
    tp_display.drawCube(tp_triad.DCM.transposed());
#elif DISPLAY_MODE == EKF_DISPLAY
    tp_display.drawCube(tp_ekf.q);
    // tp_display.draw_acc_mag_in_ball(tp_ekf.mpu.UnitAccelBody, tp_ekf.mag.UnitMagVect);
    // tp_display.draw_acc_mag_in_ball2(tp_ekf.y_a, tp_ekf.y_m);

    tp_display.printVector(tp_ekf.mpu.UnitAccelBody, 10, 90, ILI9341_BLUE);

    tp_display.printVector(tp_ekf.mag.UnitMagVect, 10, 150, ILI9341_MAGENTA);

#elif DISPLAY_MODE == MEKF_DISPLAY
    tp_display.drawCube(tp_mekf.get_q());
    tp_mekf.mag.read_mag();
    tp_display.draw_acc_mag_in_ball(tp_mekf.mpu.UnitAccelBody, tp_mekf.mag.UnitMagVect);
    // tp_display.draw_acc_mag_in_ball2(tp_ekf.y_a, tp_ekf.y_m);

    // tp_display.printVector(tp_ekf.v_a, 10, 90, ILI9341_BLUE);

    // tp_display.printVector(tp_ekf.v_m, 10, 150, ILI9341_MAGENTA);
    // tp_display.printVector(tp_ekf.mpu.GyroRate, 280, 50);
#endif


    // rot += rot*0.001;
    // if(theta>=M_2PI)
    //     theta=0;
    // theta += 0.001;
    // q.from_axis_angle(rot, theta);
    // tp_display.drawCube(q);


    //tp_display.draw_euler_deg(tp_bkf.mpu.accel_roll, tp_bkf.mpu.accel_pitch);
    
    // Serial.print(" Theta_k:");
    // Serial.println(theta_k);

    // Serial.print(", ");
    // Serial.print(theta_gyro);
    // Serial.print(", ");
    // Serial.print(theta_accel);
    // Serial.print(" K");
    // Serial.println(KalmanGain, 6);
    // Serial.print(" Var");
    // Serial.println(var_theta_k,6);

    // Serial.print(" acc_z_inert:");
    // Serial.print(acc_z_inertial);
    // Serial.print(" baro:");
    // Serial.print(alt_baro);
    // static float K11 = K(0,0);
    // static float K21 = K(1,0);
    // Serial.print("K11:");
    // Serial.print(K11,8);
    // Serial.print(" K21:");
    // Serial.print(K21,8);
    // read_mag();
    // tp_bkf.mag.read_mag();
    // Serial.print("mag0:");
    // float x, y, z;
    // DCM.to_euler(&x, &y, &z);


    // Serial.print(x);
    // Serial.print(",");
    // Serial.print(y);
    // Serial.print(",");
    // Serial.print(z);
    // Serial.print("---");
    // Serial.print(a.x);
    // Serial.print(",");
    // Serial.print(a.y);
    // Serial.print(",");
    // Serial.print(a.z);
    // Serial.print("---");

    // debug(tp_ekf.P[0]);
    // debug(tp_ekf.P[1]);
    // debug(tp_ekf.P[2]);
    // debug(tp_ekf.P[3]);
    // debug(tp_ekf.P[4]);
    // debug(tp_ekf.P[5]);
    // debug(tp_ekf.P[6]);
    // debug(tp_ekf.P[7]);
    // debug(tp_ekf.P[8]);
    // debugln(tp_ekf.P[9]);
    
    /*
    Magneto Calibrate 
    */
    // Serial.print(int(tp_triad.mag.MagRaw.x));
    // Serial.print(",");
    // Serial.print(int(tp_triad.mag.MagRaw.y));
    // Serial.print(",");
    // Serial.println(int(tp_triad.mag.MagRaw.z));
    
    
    // Serial.print(",");
    // Serial.print(tp_bkf.accel_roll);
    // Serial.print("   ");
    // Serial.println(tp_bkf.accel_pitch);
    // Serial.println("   ");

    // Serial.print(" angle_roll [deg]:");
    // Serial.print(angle_roll);
    // Serial.print(" angle_pitch [deg]:");
    // Serial.print(angle_pitch);
    // Serial.print(" angle_yaw [deg]:");
    // Serial.print(angle_yaw);
    // Serial.print(" altitude[cm]:");
    // Serial.print(altitude);
    // Serial.print(" vertical velocity[cm/s]:");
    // Serial.println(velocity_z); 


    // Serial.print("T");
    debugln(micros() - loop_timer);
#if DISPLAY_MODE != 0
    tp_display.printTime(micros() - loop_timer);
#endif

    while (micros() - loop_timer < Tus);
}
