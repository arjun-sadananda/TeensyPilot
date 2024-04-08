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

#define MAG_FOR_CALIB true

#if SERIAL_ON
    #define debug(x) Serial.print(x); Serial.print(" ");
    #define debugln(x) Serial.println(x)
#else
    #define debug(x) 
    #define debugln(x)
#endif

uint32_t start_time;
uint32_t read_time, estimate_time, display_time;
TP_ESTIMATOR tp_estimator;
TP_Display tp_display;

/*
main
*/
void setup() {

    // Serial.println("Starting Wire");
    Wire.begin();
    Wire.setClock(400000);
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
#if DISPLAY_ON
    tp_display.printStatus("Estimator Running");
#endif
}

void loop() {
    start_time = micros();

    tp_estimator.read_sensors();
    read_time = micros() - start_time;

    tp_estimator.estimate_attitude();
    estimate_time = micros() - start_time - read_time;

    // ------------------- For Serial Plotter -------------------------
#if DISPLAY_ON
    static int display_skip_count = 0;
    if(display_skip_count > 5){

#if ESTIMATOR == BKF
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
#elif ESTIMATOR == TRIAD
        tp_display.draw_acc_mag_in_ball(tp_estimator.UnitAccVect, tp_estimator.UnitMagVect);
        // tp_display.draw_acc_mag_in_ball2(tp_estimator.tp_triad.c2, tp_estimator.tp_triad.m);
        tp_display.printVector(tp_estimator.UnitAccVect, 10, 80);
        tp_display.printVector(tp_estimator.UnitMagVect, 10, 150);
        // #elif ESTIMATOR == TRIAD && DISPLAY_ON
        // tp_display.draw_acc_mag_in_ball(tp_estimator.mpu.UnitAccVect, tp_estimator.mag.UnitMagVect);
        // tp_display.draw_acc_mag_in_ball2(tp_estimator.tp_triad.c2, tp_estimator.tp_triad.m);
        // Transposed to display cude as fixed and display moving
        tp_display.drawCube(tp_estimator.tp_triad.DCM.transposed());
#elif ESTIMATOR == EKF
        tp_display.drawCube(tp_estimator.tp_ekf.q.tofloat());
        tp_display.draw_acc_mag_in_ball(tp_estimator.tp_ekf.a_m.tofloat(), tp_estimator.tp_ekf.m_m.tofloat());
        tp_display.draw_acc_mag_in_ball2(tp_estimator.tp_ekf.a_p.tofloat(), tp_estimator.tp_ekf.m_p.tofloat());

        tp_display.printVector(tp_estimator.mpu.UnitAccVect, 10, 90, ILI9341_BLUE);

        tp_display.printVector(tp_estimator.mag.UnitMagVect, 10, 150, ILI9341_MAGENTA);

#elif ESTIMATOR == MEKF_acc || ESTIMATOR == MEKF_mag
        tp_display.drawCube(tp_estimator.tp_mekf.get_q());
        tp_display.draw_acc_mag_in_ball(tp_estimator.tp_mekf.v_m, tp_estimator.tp_mekf.v_p.tofloat());
        // tp_display.draw_acc_mag_in_ball2(tp_mekf.a_p.tofloat(), tp_mekf.mag.m_ref);

        // tp_display.printVector(tp_ekf.v_a, 10, 90, ILI9341_BLUE);

        // tp_display.printVector(tp_ekf.v_m, 10, 150, ILI9341_MAGENTA);
        // tp_display.printVector(tp_ekf.mpu.GyroRate, 280, 50);
#elif ESTIMATOR == MEKF2
        tp_display.drawCube(tp_estimator.tp_mekf2.get_q());
        tp_display.draw_acc_mag_in_ball(tp_estimator.tp_mekf2.a_p.tofloat(), tp_estimator.tp_mekf2.m_p.tofloat());
        tp_display.draw_acc_mag_in_ball2(tp_estimator.tp_mekf2.a_m, tp_estimator.tp_mekf2.m_m);

        // // tp_display.printVector(tp_ekf.v_a, 10, 90, ILI9341_BLUE);

        // tp_display.printVector(tp_estimator.tp_mekf2.a_m, 10, 150, ILI9341_MAGENTA);
        // tp_display.printVector(tp_estimator.tp_mekf2.m_m, 10, 80);
#endif
        

        static Vector3l timer;
        timer.set(read_time, estimate_time, display_time);
        tp_display.printVector(timer, 50, 30);
        timer.set(int(1000000.0/read_time), int(1000000.0/estimate_time), int(1000000.0/display_time));
        tp_display.printVector(timer, 50, 42);

        display_time = micros() - start_time - read_time - estimate_time;
        // tp_display.printTime(micros() - start_time);
        // tp_display.printTime(tp_estimator.dt*1000000);
        display_skip_count = 0;
    }
    
    if(display_skip_count == 0) // ==1 to see on display interation (read+estimate+display time), 
                                // !=1 to see on other iterations(read + estimate time)
        tp_display.printTime(tp_estimator.dt*1000000);
    display_skip_count++;
#endif

#if DISPLAY_ON
    // tp_display.printTime(micros() - loop_timer);
#endif

#if MAG_FOR_CALIB
    /*
    Magneto Calibrate 
    */
    Serial.print(int(tp_estimator.MagRaw.x));
    Serial.print(",");
    Serial.print(int(tp_estimator.MagRaw.y));
    Serial.print(",");
    Serial.println(int(tp_estimator.MagRaw.z));
    while(micros() - start_time < 5000);
#endif
}
