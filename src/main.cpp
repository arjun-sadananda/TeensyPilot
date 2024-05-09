// #include <zephyr.h>
#include <Arduino.h>

#include "TP_Display\TP_Display.h"
#include "TP_StateEstimate\TP_ESTIMATOR.h"
#include "TP_Control\TP_Control.h"
#include "SD.h"
// #define DEBUG_MODE true
// #define DEBUG_AHRS true
// #define DEBUG_ALT_EST true

// #define MOTORS_ON false

#define DISPLAY_ON true
const int display_type = MEKF2_DISPLAY; 


#define SERIAL_ON false //SERIAL_DEBUG
#define MAG_FOR_CALIB false
#define SERIAL_MONITOR true
#define SERIAL_FOR_AHRSs false

// #define SERIAL_MODE SERIAL_MONITOR

#define SD_LOG_ON false

#if SERIAL_ON
    #define debug(x) Serial.print(x); Serial.print(" ");
    #define debugln(x) Serial.println(x)
#else
    #define debug(x) 
    #define debugln(x)
#endif

uint32_t start_time, control_start_time;
uint32_t read_time, estimate_time, display_time;

TP_ESTIMATOR tp_estimator;

#if DISPLAY_ON
    TP_Display tp_display;
#endif

#if SD_LOG_ON
    File myFile;
#endif

/*
    ******************************************* setup *******************************************
*/

void setup() {
    pinMode(13, OUTPUT);
    pinMode(8, INPUT_PULLUP);

    // digitalWrite(13, LOW);

#if SERIAL_ON || SERIAL_MONITOR || MAG_FOR_CALIB
    Serial.begin(115200);
#endif

#if DISPLAY_ON
    tp_display.display_setup(display_type);
    tp_display.printStatus("Sensor Initialising");
#endif

#if SD_LOG_ON
    if(!SD.begin(BUILTIN_SDCARD)){
        tp_display.printStatus("SD intialisation failed!");
    }
    tp_display.printStatus("SD intialised!");
    delay(1000);
    myFile = SD.open("test.csv", FILE_WRITE);
    if (myFile) {
        tp_display.printStatus("File opened!");
        String data = "time,gx,gy,gz,ax,ay,ax,mx,my,mz,P00,P01,P02,P03,P04,P05,P10,P11,P12,P13,P14,P15,P20,P21,P22,P23,P24,P25,P30,P31,P32,P33,P34,P35,P40,P41,P42,P43,P44,P45,P50,P51,P52,P53,P54,P55,K00,,,,,,,,,K10,,,,,,,,,K20,,,,,,,,,K30,,,,,,,,,K40,,,,,,,,,K50,,,,,,,,K58";
        data.append(",P00,P01,P02,P03,P04,P05,P10,P11,P12,P13,P14,P15,P20,P21,P22,P23,P24,P25,P30,P31,P32,P33,P34,P35,P40,P41,P42,P43,P44,P45,P50,P51,P52,P53,P54,P55,K00,,,,,,,,,K10,,,,,,,,,K20,,,,,,,,,K30,,,,,,,,,K40,,,,,,,,,K50,,,,,,,,K58");
        data.append(",P00,P01,P02,P03,P04,P05,P10,P11,P12,P13,P14,P15,P20,P21,P22,P23,P24,P25,P30,P31,P32,P33,P34,P35,P40,P41,P42,P43,P44,P45,P50,P51,P52,P53,P54,P55,K00,,,,,,,,,K10,,,,,,,,,K20,,,,,,,,,K30,,,,,,,,,K40,,,,,,,,,K50,,,,,,,,K58");
        myFile.println(data);
        delay(1000);
    }
#endif

    debugln("Init and Calib Sensors");
#if DISPLAY_ON
    tp_display.printStatus("Sensor & Estimator Initialising");
#endif

    tp_estimator.init_sensors();
    tp_estimator.init_estimator();

#if DISPLAY_ON
    tp_display.printStatus("ESC Initialising");
#endif

    ESC_setup();

    // for(int i = 0; i<300; i+=1){
    //     set_motor_speeds(1000+i, 1000+i, 1000+i, 1000+i);
    //     delay(10);
    // }
    set_motor_speeds(1000, 1000, 1000, 1000);
    control_start_time = micros();
#if DISPLAY_ON
    tp_display.printStatus("Main Loop Running");
#endif
}

bool ready = false;
const int throw_delay = 200;

/*
    ******************************************* loop *******************************************
*/

void loop() {

    start_time = micros();

    tp_estimator.read_sensors();
    read_time = micros() - start_time; 

    tp_estimator.estimate_attitude();
    estimate_time = micros() - start_time - read_time;

    if (digitalRead(8) == LOW)  // Button Pressed
        ready = true;
    if (ready == true && digitalRead(8) == HIGH){   // Button Released
        control_start_time = micros();
        ready = false;
        delay(throw_delay);
    }

    static Vector3f M, M2;
    float f;
    const float m = 1, g=1;
    int v[4];
    static Matrix3f R;
    tp_estimator.q.rotation_matrix(R);
    M = SO3_attitude_control( R, tp_estimator.omega);
    M2 = euler_attitude_control( R, tp_estimator.omega);
    f = m*g*R.c.z;
    if((micros() - control_start_time)*1000000 < 10 && ready == false){ // Button Not Pressed
        // command_motors(f, M, v);
    }
    else{
        v[0] = v[1] = v[2] = v[3] = 1000;
        set_motor_speeds(1000, 1000, 1000, 1000);
    }


#if SD_LOG_ON
    static int sd_log_count = 0;
    static String data;
    if(sd_log_count < 110 && myFile){
        data = String(start_time);
                    // + "," + String(tp_estimator.GyroRate.x, 6) 
                    // + "," + String(tp_estimator.GyroRate.y, 6) 
                    // + "," + String(tp_estimator.GyroRate.z, 6)
                    // + "," + String(tp_estimator.UnitAccVect.x, 6) 
                    // + "," + String(tp_estimator.UnitAccVect.y, 6) 
                    // + "," + String(tp_estimator.UnitAccVect.z, 6)
                    // + "," + String(tp_estimator.UnitMagVect.x, 6) 
                    // + "," + String(tp_estimator.UnitMagVect.y, 6) 
                    // + "," + String(tp_estimator.UnitMagVect.z, 6);
        for(int i =0; i<36; i++)
                data.append("," + String(tp_estimator.tp_mekf2.P[i] , 6));
        for(int i = 0; i<6; i++)
            for(int j =0; j<9; j++)
                data.append("," + String(tp_estimator.tp_mekf2.K[i*9+j], 6));

        for(int i =0; i<36; i++)
                data.append("," + String(tp_estimator.tp_mekf2_acc.P[i] , 6));
        for(int i = 0; i<6; i++)
            for(int j =0; j<9; j++)
                data.append("," + String(tp_estimator.tp_mekf2_acc.K[i*9+j], 6));

        for(int i =0; i<36; i++)
                data.append("," + String(tp_estimator.tp_mekf2_triad.P[i] , 6));
        for(int i = 0; i<6; i++)
            for(int j =0; j<9; j++)
                data.append("," + String(tp_estimator.tp_mekf2_triad.K[i*9+j], 6));

        myFile.println(data);
        sd_log_count ++;
    }
    if(sd_log_count == 110){
        myFile.close();
        tp_display.printStatus("Done Logging");
        sd_log_count ++;
    }
#endif


    // ------------------- For Serial Plotter -------------------------
#if DISPLAY_ON
    static int display_skip_count = 0;
    if(display_skip_count > 50){

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
#elif ESTIMATOR == MEKF2 || ESTIMATOR == ALL_ESTIMATORS
        tp_display.drawCube(tp_estimator.q);
        tp_display.draw_acc_mag_in_ball(tp_estimator.tp_mekf2.a_p.tofloat(), tp_estimator.tp_mekf2.m_p.tofloat());
        tp_display.draw_acc_mag_in_ball2(tp_estimator.tp_mekf2.a_m, tp_estimator.tp_mekf2.m_m);

        tp_display.printStatus(tp_estimator.tp_mekf2.get_a_res_norm()*10000);
        tp_display.printStatus(tp_estimator.tp_mekf2.get_m_res_norm()*10000, 150);
        // // tp_display.printVector(tp_ekf.v_a, 10, 90, ILI9341_BLUE);

        tp_display.printVector(tp_estimator.tp_mekf2.a_m, 10, 150, ILI9341_MAGENTA);
        tp_display.printVector(tp_estimator.tp_mekf2.m_m, 10, 80);
        
#elif ESTIMATOR == MEKF2_TRIAD || ESTIMATOR == MEKF2_COMPARE
        tp_display.drawCube(tp_estimator.q);
        tp_display.draw_acc_mag_in_ball(tp_estimator.tp_mekf2_triad.a_p.tofloat(), tp_estimator.tp_mekf2_triad.m_p.tofloat());
        tp_display.draw_acc_mag_in_ball2(tp_estimator.tp_mekf2_triad.a_m, tp_estimator.tp_mekf2_triad.m_m);

        tp_display.printStatus(tp_estimator.tp_mekf2_triad.get_a_res_norm()*10000);
        tp_display.printStatus(tp_estimator.tp_mekf2_triad.get_m_res_norm()*10000, 150);
        // // tp_display.printVector(tp_ekf.v_a, 10, 90, ILI9341_BLUE);

        tp_display.printVector(tp_estimator.tp_mekf2_triad.a_m, 10, 150, ILI9341_MAGENTA);
        tp_display.printVector(tp_estimator.tp_mekf2_triad.m_m, 10, 80);

        tp_display.printVector(M, 110, 80, ILI9341_ORANGE);
        tp_display.printVector(M2, 140, 80, ILI9341_ORANGE);
        tp_display.printStatus(f*100, 110, 60);
        tp_display.printVel(v, 110, 150, ILI9341_ORANGE);
        
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

#if SERIAL_MONITOR
    static int serial_skip_count = 0;
    if(serial_skip_count > 50){
        static Quaternion q;
        q = tp_estimator.tp_mekf2.get_q();
        Serial.print(q.q1);        Serial.print(",");
        Serial.print(q.q2);        Serial.print(",");
        Serial.print(q.q3);        Serial.print(",");
        Serial.print(q.q4);        Serial.print(",");

        q = tp_estimator.tp_mekf2_acc.get_q();
        Serial.print(q.q1);        Serial.print(",");
        Serial.print(q.q2);        Serial.print(",");
        Serial.print(q.q3);        Serial.print(",");
        Serial.print(q.q4);        Serial.print(",");

        q = tp_estimator.tp_mekf2_triad.get_q();
        Serial.print(q.q1);        Serial.print(",");
        Serial.print(q.q2);        Serial.print(",");
        Serial.print(q.q3);        Serial.print(",");
        Serial.print(q.q4);        Serial.print(",");


        static Vector3f a_m, m_m, a_p, m_p;

        a_m = tp_estimator.tp_mekf2.a_m;
        m_m = tp_estimator.tp_mekf2.m_m;
        a_p = tp_estimator.tp_mekf2.a_p.tofloat();
        m_p = tp_estimator.tp_mekf2.m_p.tofloat();

        Serial.print(a_m.x);        Serial.print(",");
        Serial.print(a_m.y);        Serial.print(",");
        Serial.print(a_m.z);        Serial.print(",");

        Serial.print(m_m.x);        Serial.print(",");
        Serial.print(m_m.y);        Serial.print(",");
        Serial.print(m_m.z);        Serial.print(","); 

        Serial.print(a_p.x);        Serial.print(",");
        Serial.print(a_p.y);        Serial.print(",");
        Serial.print(a_p.z);        Serial.print(",");

        Serial.print(m_p.x);        Serial.print(",");
        Serial.print(m_p.y);        Serial.print(",");
        Serial.print(m_p.z);        Serial.print(","); 

        a_m = tp_estimator.tp_mekf2_acc.a_m;
        m_m = tp_estimator.tp_mekf2_acc.m_m;
        a_p = tp_estimator.tp_mekf2_acc.a_p.tofloat();
        m_p = tp_estimator.tp_mekf2_acc.m_p.tofloat();

        Serial.print(a_m.x);        Serial.print(",");
        Serial.print(a_m.y);        Serial.print(",");
        Serial.print(a_m.z);        Serial.print(",");

        Serial.print(m_m.x);        Serial.print(",");
        Serial.print(m_m.y);        Serial.print(",");
        Serial.print(m_m.z);        Serial.print(","); 

        Serial.print(a_p.x);        Serial.print(",");
        Serial.print(a_p.y);        Serial.print(",");
        Serial.print(a_p.z);        Serial.print(",");

        Serial.print(m_p.x);        Serial.print(",");
        Serial.print(m_p.y);        Serial.print(",");
        Serial.print(m_p.z);        Serial.print(","); 

        a_m = tp_estimator.tp_mekf2_triad.a_m;
        m_m = tp_estimator.tp_mekf2_triad.m_m;
        a_p = tp_estimator.tp_mekf2_triad.a_p.tofloat();
        m_p = tp_estimator.tp_mekf2_triad.m_p.tofloat();

        Serial.print(a_m.x);        Serial.print(",");
        Serial.print(a_m.y);        Serial.print(",");
        Serial.print(a_m.z);        Serial.print(",");

        Serial.print(m_m.x);        Serial.print(",");
        Serial.print(m_m.y);        Serial.print(",");
        Serial.print(m_m.z);        Serial.print(","); 

        Serial.print(a_p.x);        Serial.print(",");
        Serial.print(a_p.y);        Serial.print(",");
        Serial.print(a_p.z);        Serial.print(",");

        Serial.print(m_p.x);        Serial.print(",");
        Serial.print(m_p.y);        Serial.print(",");
        Serial.println(m_p.z); 

        serial_skip_count = 0;
    }
    serial_skip_count++;
#endif
#if SERIAL_FOR_AHRSs
    static int serial_skip_count = 0;
    if(serial_skip_count > 50){
        static Quaternion q;
        q.from_rotation_matrix(tp_estimator.tp_triad.DCM);
        Serial.print(q.q1);        Serial.print(",");
        Serial.print(q.q2);        Serial.print(",");
        Serial.print(q.q3);        Serial.print(",");
        Serial.print(q.q4);        Serial.print(",");

        q = tp_estimator.tp_mekf.get_q();
        Serial.print(q.q1);        Serial.print(",");
        Serial.print(q.q2);        Serial.print(",");
        Serial.print(q.q3);        Serial.print(",");
        Serial.print(q.q4);        Serial.print(",");

        q = tp_estimator.tp_mekf2.get_q();
        Serial.print(q.q1);        Serial.print(",");
        Serial.print(q.q2);        Serial.print(",");
        Serial.print(q.q3);        Serial.print(",");
        Serial.print(q.q4);        Serial.print(",");

        q = tp_estimator.tp_mekf2_triad.get_q();
        Serial.print(q.q1);        Serial.print(",");
        Serial.print(q.q2);        Serial.print(",");
        Serial.print(q.q3);        Serial.print(",");
        Serial.println(q.q4);
        
        serial_skip_count = 0;
    }
    serial_skip_count++;
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


}


