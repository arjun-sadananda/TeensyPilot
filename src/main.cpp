// #include <zephyr.h>
#include <Arduino.h>

#include "AP_Math.h"
// #include "TP_Display\TP_Display.h"
#include "TP_StateEstimate\TP_ESTIMATOR.h"
#include "Wire.h"

// #include "teensyshot/ESCCMD.h"
// #include "teensyshot/DSHOT.h"

#include <Servo.h>

// #define DEBUG_MODE true
// #define DEBUG_AHRS true
// #define DEBUG_ALT_EST true

#define DISPLAY_ON false
// const int display_type = MEKF2_DISPLAY; 
#define SERIAL_ON false

#define MAG_FOR_CALIB false

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

int16_t speed = 0;
// TP_Display tp_display;

/*
main
*/
// FlexPWM4.0	22
// FlexPWM4.1	23	4.482 kHz
// FlexPWM4.2	2, 3
#define ESCPIN1 2
#define ESCPIN2 3
#define ESCPIN3 4
#define ESCPIN4 5

#define ESC_CALIB false
Servo ESC1, ESC2, ESC3, ESC4;

void setup() {
    pinMode(13, OUTPUT);
    ESC1.attach(ESCPIN1, 1000, 2000);
    ESC2.attach(ESCPIN2, 1000, 2000);
    ESC3.attach(ESCPIN3, 1000, 2000);
    ESC4.attach(ESCPIN4, 1000, 2000);

    ESC1.writeMicroseconds(1000);
    ESC2.writeMicroseconds(1000);
    ESC3.writeMicroseconds(1000);
    ESC4.writeMicroseconds(1000); // 3 beeps - powering on
    delay(3000);

    ESC1.writeMicroseconds(1200);
    ESC2.writeMicroseconds(1200);
    ESC3.writeMicroseconds(1200);
    ESC4.writeMicroseconds(1200);   // 1 long low beep - arming sequence begins
    delay(1500);
#if ESC_CALIB
    // Calibration 
    // ESC1.writeMicroseconds(1110);
    // ESC1.writeMicroseconds(2230);
    ESC1.writeMicroseconds(2000);
    ESC2.writeMicroseconds(2000);
    ESC3.writeMicroseconds(2000);
    ESC4.writeMicroseconds(2000);   // single highs beeps - measure
    delay(7000);                   // rising beeps - calibration done

    ESC1.writeMicroseconds(1000);
    ESC2.writeMicroseconds(1000);
    ESC3.writeMicroseconds(1000);
    ESC4.writeMicroseconds(1000);   // double low beeps - measure
    delay(7000);                   // falling beeps - calibration done
    // Calibration Ended
#endif
    ESC1.writeMicroseconds(1000);
    ESC2.writeMicroseconds(1000);
    ESC3.writeMicroseconds(1000);
    ESC4.writeMicroseconds(1000);   // 1 long high beep (if not callibrating) - arming sequence ends
    delay(5000);
    for(int i = 0; i<300; i+=1){
        ESC1.writeMicroseconds(1000+i);
        ESC2.writeMicroseconds(1000+i);
        ESC3.writeMicroseconds(1000+i);
        ESC4.writeMicroseconds(1000+i);
        delay(10);
        // For searching for max min if calibrated unkown value by mistake (uing analogWrite() etc.!!
        // Serial.println(i);
        // digitalWrite(13,HIGH);
        // delay(100);
        // digitalWrite(13,LOW);
        // delay(1000);
    }
    ESC1.writeMicroseconds(1000);
    ESC2.writeMicroseconds(1000);
    ESC3.writeMicroseconds(1000);
    ESC4.writeMicroseconds(1000);

    // DSHOT_init(count);
    // ret = DSHOT_arm();
    // ret = ESCCMD_beep(); 

    // TCCR1
    // ESC.attach(4,1000,2000);
    // ESC.writeMicroseconds(0);
    // delay(1000);
    // ESC.writeMicroseconds(45);
    // delay(1);
    // ESC.writeMicroseconds(0);
    // delay(1);
    // ESC.writeMicroseconds(45);
    // delay(2000);
    // Serial.println("Starting Wire");
    // Wire.begin();
    // Wire.setClock(400000);
    // delay(250);
    
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
    
    // tp_estimator.init_sensors();
    // tp_estimator.init_estimator();
#if DISPLAY_ON
    tp_display.printStatus("Estimator Running");
#endif

    // delay(500);
    // int count = 6, ret;

//
// *************************************************************************
//

    // Initialize the CMD subsystem
    // tp_display.printStatus("Initializing DSHOT");
    // ESCCMD_init(count);//ESCPID_NB_ESC );
    // delay(4000);

    // Arming ESCs
    // tp_display.printStatus("Arming All");
    // ret = ESCCMD_arm_all( );
    // ret = DSHOT_arm();
    // tp_display.printStatus(ret);
    // delay(2000);

    // Switch 3D mode on
    // tp_display.printStatus("Beep");
    // ret = ESCCMD_beep(); 
    // tp_display.printStatus(ret);
    // delay(2000);

    // tp_display.printStatus("3d off");
    // // Switch 3D mode on
    // ESCCMD_3D_off(); 
    // delay(2000);

    // tp_display.printStatus("Arming All");
    // // Arming ESCs
    // ESCCMD_arm_all( );
    // delay(3000);


    // tp_display.printStatus("start timer");
    // Start periodic loop
    // ret = ESCCMD_start_timer( );
    // tp_display.printStatus(ret);
    // delay(2000);


    // tp_display.printStatus("stop all");
    // Stop all motors
    // for ( int i = 0; i < count; i++ ) {//ESCPID_NB_ESC
        // ESCCMD_stop( i );
        // analogWrite(4, 100);
        // delay(3000);
        // analogWrite(4, 0);
    // }
    // delay(2000);

    // tp_display.printStatus("Estimator Running");
    // startDshot(); // sets the DMA up and disables telemetry for initialization

    // armingMotor(); // arms the motor by sending 0 command at least 10 times
    // delay(500); // for debug - giving time to esc to say "ARMED"

    // disable3d(); // enables the 3D flight mode
    // setControlLoopFrequency(DSHOT_FREQ); // sets the control loop frequency for rpm control
    // disableTlm(); // enables the telemetry
    speed = 0;
}

void loop() {
    // start_time = micros();
    // // if (speed < 1000)
    //     speed++;
    // // else
    //     // speed = 0;
    // // ESCCMD_beep();
    // static int    i, ret;
    // // static uint16_t            cmd[4]; 
    // if (speed < 10000) ret = ESCCMD_tic( );
    
    // if ( speed < 10000 ){//&& ret == ESCCMD_TIC_OCCURED )  {
    //     for ( i = 0; i < 4; i++ ) {
    // //         // cmd[i] = DSHOT_CMD_MAX + 1 + 300;
    //     // if(speed >100000)
    //         ret = ESCCMD_throttle( i, (int16_t)100);
    // //         ESCCMD_stop(i);
            
    // //         // tp_display.printStatus(ret);
    //     }
    //     // DSHOT_send(cmd, 0);
    //     // ret = ESCCMD_beep(); 
    // }
    // if (speed > 10000){
    //         for ( int i = 0; i < 4; i++ ) //ESCPID_NB_ESC
    //     ESCCMD_stop( i );
    // }

    while(micros() - start_time < 1000);

/*
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
        // q.from_euler(rot);// *AP_DEG_TO_RAD);
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
    //Magneto Calibrate 
    Serial.print(int(tp_estimator.MagRaw.x));
    Serial.print(",");
    Serial.print(int(tp_estimator.MagRaw.y));
    Serial.print(",");
    Serial.println(int(tp_estimator.MagRaw.z));
    while(micros() - start_time < 5000);
#endif
*/
}


