// #include <zephyr.h>
#include <Arduino.h>

#include "AP_Math.h"
#include "TP_Display\TP_Display.h"
#include "TP_StateEstimate\TP_ESTIMATOR.h"
#include "Wire.h"

// #define DEBUG_MODE true
// #define DEBUG_AHRS true
// #define DEBUG_ALT_EST true

#define DISPLAY_ON true
const int display_type = MEKF2_DISPLAY; 
#define SERIAL_ON false

#if SERIAL_ON
    #define debug(x) Serial.print(x); Serial.print(" ");
    #define debugln(x) Serial.println(x)
#else
    #define debug(x) 
    #define debugln(x)
#endif

uint32_t loop_timer;
TP_ESTIMATOR tp_estimator;
TP_Display tp_display;

int display_skip_count = 0;
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

#if DISPLAY_ON
    tp_display.display_setup(display_type);
    tp_display.printStatus("Calibrating Gyro");
#endif
    debugln("Init and Calib Sensors");
    
    tp_estimator.init_sensors();
    tp_estimator.init_estimator();

    tp_display.printStatus("Estimator Running");

}

void loop() {
    loop_timer = micros();

    tp_estimator.read_sensors();
    tp_estimator.estimate_attitude();

    // ------------------- For Serial Plotter -------------------------
    if(display_skip_count > 5){
#if ESTIMATOR == BKF && DISPLAY_ON
    tp_display.draw_euler_deg(tp_estimator.tp_bkf.attitude_euler.x, tp_estimator.tp_bkf.attitude_euler.y, tp_estimator.tp_bkf.attitude_euler.z);
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
    tp_display.drawCube(tp_estimator.tp_bkf.accel_roll, tp_estimator.tp_bkf.accel_pitch, 0);
    debug(tp_estimator.tp_bkf.accel_roll);
    debug(tp_estimator.tp_bkf.accel_pitch);
    debugln(tp_estimator.tp_bkf.mag_yaw);
#elif ESTIMATOR == TRIAD && DISPLAY_ON
    tp_display.draw_acc_mag_in_ball(tp_estimator.mpu.UnitAccelBody, tp_estimator.mag.UnitMagVect);
    tp_display.draw_acc_mag_in_ball2(tp_estimator.tp_triad.c2, tp_estimator.tp_triad.m);
    tp_display.printVector(tp_estimator.mpu.UnitAccelBody, 10, 80);
    tp_display.printVector(tp_estimator.mag.UnitMagVect, 10, 150);
// #elif ESTIMATOR == TRIAD && DISPLAY_ON
    // tp_display.draw_acc_mag_in_ball(tp_estimator.mpu.UnitAccelBody, tp_estimator.mag.UnitMagVect);
    // tp_display.draw_acc_mag_in_ball2(tp_estimator.tp_triad.c2, tp_estimator.tp_triad.m);
    // Transposed to display cude as fixed and display moving
    tp_display.drawCube(tp_estimator.tp_triad.DCM.transposed());
#elif ESTIMATOR == EKF && DISPLAY_ON
    tp_display.drawCube(tp_estimator.tp_ekf.q.tofloat());
    tp_display.draw_acc_mag_in_ball(tp_estimator.tp_ekf.a_m.tofloat(), tp_estimator.tp_ekf.m_m.tofloat());
    tp_display.draw_acc_mag_in_ball2(tp_estimator.tp_ekf.a_p.tofloat(), tp_estimator.tp_ekf.m_p.tofloat());

    tp_display.printVector(tp_estimator.mpu.UnitAccelBody, 10, 90, ILI9341_BLUE);

    tp_display.printVector(tp_estimator.mag.UnitMagVect, 10, 150, ILI9341_MAGENTA);

#elif (ESTIMATOR == MEKF_acc || ESTIMATOR == MEKF_mag) && DISPLAY_ON
    tp_display.drawCube(tp_estimator.tp_mekf.get_q());
    tp_display.draw_acc_mag_in_ball(tp_estimator.tp_mekf.v_m, tp_estimator.tp_mekf.v_p.tofloat());
    // tp_display.draw_acc_mag_in_ball2(tp_mekf.a_p.tofloat(), tp_mekf.mag.m_ref);

    // tp_display.printVector(tp_ekf.v_a, 10, 90, ILI9341_BLUE);

    // tp_display.printVector(tp_ekf.v_m, 10, 150, ILI9341_MAGENTA);
    // tp_display.printVector(tp_ekf.mpu.GyroRate, 280, 50);
#elif ESTIMATOR == MEKF2 && DISPLAY_ON
    tp_display.drawCube(tp_estimator.tp_mekf2.get_q());
    tp_display.draw_acc_mag_in_ball(tp_estimator.tp_mekf2.a_p.tofloat(), tp_estimator.tp_mekf2.m_p.tofloat());
    tp_display.draw_acc_mag_in_ball2(tp_estimator.tp_mekf2.a_m, tp_estimator.tp_mekf2.m_m);

    // tp_display.printVector(tp_ekf.v_a, 10, 90, ILI9341_BLUE);

    tp_display.printVector(tp_estimator.tp_mekf2.a_m, 10, 150, ILI9341_MAGENTA);
    tp_display.printVector(tp_estimator.tp_mekf2.m_m, 10, 80);
#endif
    display_skip_count = 0;
    }
    display_skip_count++;
    // rot += rot*0.001;
    // if(theta>=M_2PI)
    //     theta=0;
    // theta += 0.001;
    // q.from_axis_angle(rot, theta);
    // tp_display.drawCube(q);

    // for(int i=0; i<54; i++){
    //     debug(tp_mekf2.K[i]);
    // }
    // for(int i=0; i<54; i++){
    //     debug(tp_mekf2.K[i]);
    // }
    // debugln();
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
    // debugln(micros() - loop_timer);
#if DISPLAY_ON
    // tp_display.printTime(micros() - loop_timer);
    
    tp_display.printTime(tp_estimator.dt*1000000);
#endif

    // while(micros() - loop_timer < 5000);

}
