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
#include "TP_Display\TP_Display.h"
#include "Wire.h"

// #define DEBUG_MODE true
// #define DEBUG_AHRS true
// #define DEBUG_ALT_EST true

#define BKF_DIPLAY true

#define BKF 0
#define TRIAD 1 
#define EKF 2

#define TP_ESTIMATOR TRIAD

#define NO_DISPLAY 0
#define ACC_MAG_BALL 1
#define BKF_DISPLAY 2
#define TRIAD_DISPLAY 3

#define DISPLAY_MODE TRIAD_DISPLAY

#define SERIAL_ON true

#if SERIAL_ON
    #define debug(x) Serial.print(x)
    #define debugln(x) Serial.println(x)
#else
    #define debug(x) 
    #define debugln(x)
#endif

uint32_t loop_timer;

#define Tus 10000

TP_BKF tp_bkf;
TP_TRIAD tp_triad;
TP_Display tp_display;
/*
main
*/
void setup() {

    // Serial.println("Starting Wire");
    Wire.setClock(400000);
    Wire.begin();
    delay(250);

#if SERIAL_ON
    Serial.begin(57600);
#endif

#if TP_ESTIMATOR == BKF
    tp_bkf.init_sensors();
#elif TP_ESTIMATOR ==TRIAD
    tp_triad.init_sensors();
#endif

#if DISPLAY_MODE == BKF_DISPLAY
    tp_display.euler_display_setup();
#elif DISPLAY_MODE == ACC_MAG_BALL
    tp_display.mag_acc_display_setup();
#elif DISPLAY_MODE == TRIAD_DISPLAY
    tp_display.triad_display_setup();
#endif

    // pinMode(13, OUTPUT);
    // digitalWrite(13, HIGH);

    debugln("Calibrating Gyro");
    tp_display.printStatus("Calibrating Gyro");
    tp_bkf.mpu.calibrate_gyro();

    // debugln("Calibrating Baro");
    // tp_display.printStatus("Calibrating Baro");
    // tp_bkf.baro.calibrate_baro();
    // Serial.println(BaroAltOffset);
    // Serial.println("Baro Calibration Done");
    tp_display.printStatus("Estimator Running");
}

void loop() {
    loop_timer = micros();

#if TP_ESTIMATOR == BKF
    tp_bkf.attitude_estimate();
    tp_bkf.vert_state_estimate();
#elif TP_ESTIMATOR == TRIAD
    tp_triad.estimate();
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
    tp_display.drawCube(tp_bkf.accel_roll, tp_bkf.accel_pitch, tp_bkf.mag_yaw);
    debug(tp_bkf.accel_roll);
    debug(tp_bkf.accel_pitch);
    debugln(tp_bkf.mag_yaw);
#elif DISPLAY_MODE == ACC_MAG_BALL
    Vector3f temp1 = tp_bkf.mpu.AccelBody, temp2 = tp_bkf.mag.MagVect;;
    temp1.rotate(ROTATION_ROLL_180_YAW_90);
    temp2.rotate(ROTATION_ROLL_180);
    tp_display.draw_acc_mag_in_ball(temp1, temp2);
#elif DISPLAY_MODE == TRIAD_DISPLAY
    tp_display.draw_acc_mag_in_ball(tp_triad.mpu.UnitAccelBody, tp_triad.mag.UnitMagVect);
    // Transposed to display cude as fixed and display moving
    tp_display.drawCube(tp_triad.DCM.transposed());
#endif
    tp_display.printTime(micros() - loop_timer);


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
    // Serial.print(m.x);
    // Serial.print(",");
    // Serial.print(m.y);
    // Serial.print(",");
    // Serial.print(m.z);
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
    // Serial.print(micros() - loop_timer);

    while (micros() - loop_timer < Tus)
    ;
}
