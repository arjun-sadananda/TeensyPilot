// #include <zephyr.h>
#include <Arduino.h>

#include "TP_Display.h"
#include "TP_Serial.h"
#include "TP_StateEstimate\TP_ESTIMATOR.h"
#include "TP_Control\TP_Control.h"
#include "SD.h"
// #define DEBUG_MODE true
// #define DEBUG_AHRS true
// #define DEBUG_ALT_EST true

// #define MOTORS_ON false

#define display_type  DRONE_MONITOR
#define serial_type SERIAL_OFF

#define SD_LOG_ON false

#if serial_type == SERIAL_DEBUG
    #define debug(x) Serial.print(x); Serial.print(" ");
    #define debugln(x) Serial.println(x)
#else
    #define debug(x) 
    #define debugln(x)
#endif

uint32_t start_time, control_start_time;
uint32_t read_time, estimate_time, display_time;//, control_time;

bool ready = false;
bool controller_on = false;
const int throw_delay = 200;

TP_ESTIMATOR tp_estimator;
TP_Control tp_control;

#if display_type != DISPLAY_OFF
    TP_Display tp_display;
    #define displayStatus(x) tp_display.printStatus(x);
#else
    #define displayStatus(x)
#endif

#if SD_LOG_ON
    File myFile;
#endif


int main(void)
{
	pinMode(13, OUTPUT);
    pinMode(8, INPUT_PULLUP);

    // digitalWrite(13, LOW);

#if serial_type != SERIAL_OFF
    Serial.begin(115200);
#endif

#if display_type != DISPLAY_OFF
    tp_display.display_setup(display_type);
#endif

#if SD_LOG_ON
    if(!SD.begin(BUILTIN_SDCARD)){
        displayStatus("SD intialisation failed!");
    }
    displayStatus("SD intialised!");
    delay(1000);
    myFile = SD.open("test.csv", FILE_WRITE);
    if (myFile) {
        displayStatus("File opened!");
        String data = "time,gx,gy,gz,ax,ay,ax,mx,my,mz,P00,P01,P02,P03,P04,P05,P10,P11,P12,P13,P14,P15,P20,P21,P22,P23,P24,P25,P30,P31,P32,P33,P34,P35,P40,P41,P42,P43,P44,P45,P50,P51,P52,P53,P54,P55,K00,,,,,,,,,K10,,,,,,,,,K20,,,,,,,,,K30,,,,,,,,,K40,,,,,,,,,K50,,,,,,,,K58";
        data.append(",P00,P01,P02,P03,P04,P05,P10,P11,P12,P13,P14,P15,P20,P21,P22,P23,P24,P25,P30,P31,P32,P33,P34,P35,P40,P41,P42,P43,P44,P45,P50,P51,P52,P53,P54,P55,K00,,,,,,,,,K10,,,,,,,,,K20,,,,,,,,,K30,,,,,,,,,K40,,,,,,,,,K50,,,,,,,,K58");
        data.append(",P00,P01,P02,P03,P04,P05,P10,P11,P12,P13,P14,P15,P20,P21,P22,P23,P24,P25,P30,P31,P32,P33,P34,P35,P40,P41,P42,P43,P44,P45,P50,P51,P52,P53,P54,P55,K00,,,,,,,,,K10,,,,,,,,,K20,,,,,,,,,K30,,,,,,,,,K40,,,,,,,,,K50,,,,,,,,K58");
        myFile.println(data);
        delay(1000);
    }
#endif

    debugln("Sensor Initialising");
    displayStatus("Sensor Initialising");
    tp_estimator.init_sensors();
    debugln("Sensor Calibration");
    displayStatus("Sensor Calibration");
    delay(2000);
    displayStatus("Recording...");
    tp_estimator.calibrate_sensors();
    debugln("Estimator Initialising");
    displayStatus("Estimator Initialising");
    tp_estimator.init_estimator();

    displayStatus("ESC Initialising");

    tp_control.init_motors();

    // tp_control.motor_test();

    tp_control.stop_motors();

    // control_start_time = micros();

    displayStatus("Main Loop Running");


    /*
     * Main Loop
     *
     */

	while(1){
		start_time = micros();

        /*
         * Read Sensors
         */
        tp_estimator.read_sensors();
        read_time = micros() - start_time; 

        /*
         * Run Attitude Estimator
         */
        tp_estimator.estimate_attitude();
        estimate_time = micros() - start_time - read_time;

        /*
         * Controller Switch: Throw Trigger
         */
        if (digitalRead(8) == LOW){  // Button Pressed
            ready = true;
            controller_on = false;
        }
        if (ready == true && digitalRead(8) == HIGH){   // Button Released
            ready = false;
            displayStatus("Controller Running");
            if (!controller_on)
                delay(throw_delay);
            control_start_time = micros();
            tp_control.clear_integral();
            controller_on = true;
        }

        
        
        // M2 = euler_attitude_control( R, tp_estimator.omega);
        
        if(int((micros() - control_start_time)/1000000.0) < 6 && controller_on == true){ // Button Not Pressed
            tp_control.throw_mode_controller(tp_estimator.q, tp_estimator.omega, true);
        }
        else{
            controller_on = false;
            tp_control.throw_mode_controller(tp_estimator.q, tp_estimator.omega, false);
            tp_control.stop_motors();
            // displayStatus("Controller Stopped");
        }
        // control_time = micros() - start_time - read_time - estimate_time;

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
#if display_type != DISPLAY_OFF
        static int display_skip_count = 0;
        if(display_skip_count > 50){

    #if display_type   == BKF_DISPLAY
        #if ESTIMATOR == BKF
            tp_display.draw_euler_deg(tp_estimator.tp_bkf.attitude_euler.x, tp_estimator.tp_bkf.attitude_euler.y, tp_estimator.tp_bkf.attitude_euler.z);
            tp_display.drawCube(tp_estimator.tp_bkf.accel_roll, tp_estimator.tp_bkf.accel_pitch, 0);
            debug(tp_estimator.tp_bkf.accel_roll);
            debug(tp_estimator.tp_bkf.accel_pitch);
            debugln(tp_estimator.tp_bkf.mag_yaw);
        #endif
    #elif display_type == TWO_NEEDLE_BALL
            tp_display.draw_acc_mag_in_ball(tp_estimator.UnitAccVect, tp_estimator.UnitMagVect);
    #elif display_type == CUBE

    #elif display_type == BALL_AND_CUBE || display_type == DRONE_MONITOR
            // if ESTIMATOR is MEKF_acc OR MEKF_mag, magnetometer vector will be jsut be zero.
            tp_display.draw_acc_mag_in_ball(tp_estimator.UnitAccVect, tp_estimator.UnitMagVect);
            tp_display.draw_acc_mag_in_ball2(tp_estimator.q.inverse().ret_earth_to_body(tp_estimator.a_ref),
                                            tp_estimator.q.inverse().ret_earth_to_body(tp_estimator.m_ref));
            tp_display.drawCube(tp_estimator.q);
            // tp_display.printVector(tp_estimator.UnitAccVect, 10, 80);
            // tp_display.printVector(tp_estimator.UnitMagVect, 10, 150);

            // tp_display.printStatus(tp_estimator.tp_mekf2.get_a_res_norm()*10000);
            // tp_display.printStatus(tp_estimator.tp_mekf2.get_m_res_norm()*10000, 150);
    #endif
        
            
            /*
             * Display control related values
             */
            // tp_display.printVector(tp_control.M, 110, 80, ILI9341_ORANGE);
            // tp_display.printVector(M2, 140, 80, ILI9341_ORANGE);
            // tp_display.printStatus(tp_control.f*100, 110, 60);
            tp_display.displayfM(tp_control.f, tp_control.M, tp_control.get_f_cap());
            // tp_display.printVel(tp_control.v, 110, 150, ILI9341_ORANGE);
            tp_display.displayVel(tp_control.v, tp_control.get_max_vel());

            /*
             * Display time for different processes
             * Sensor Reading, Estimator dt value, display time
             */
            display_time = micros() - start_time - read_time - estimate_time;// - control_time;
            static Vector3l timer;
            timer.set(read_time, estimate_time, display_time);
            tp_display.printVector(timer, 50, 30);
            /* Rate in Hz for seperate tasks */
            // timer.set(int(1000000.0/read_time), int(1000000.0/estimate_time), int(1000000.0/display_time));
            // tp_display.printVector(timer, 50, 42);




            display_skip_count = 0;
        }
            // outside this block if display_skip_count is  
            //                              ==1 display iteration (read+estimate+display time), 
            //                              !=1 display other iterations(read+estimate time)
        if(display_skip_count == 1)
            tp_display.printTime((micros()-start_time));
            // tp_display.printTime(tp_estimator.dt*1000000);
        display_skip_count++;
#endif

#if serial_type != SERIAL_OFF
        static int serial_skip_count = 0;
        if(serial_skip_count > 50){
    #if serial_type == MEKF2_COMPARE_MONITOR
            MEKF2_COMPARE_print(tp_estimator);

    #elif serial_type == ALL_ESTIMATORS_MONITOR
            ALL_ESTIMATORS_print(tp_estimator);
            
    #elif serial_type == MAG_FOR_CALIB
            //Magneto Calibrate 
            Serial.print(int(tp_estimator.MagRaw.x));
            Serial.print(",");
            Serial.print(int(tp_estimator.MagRaw.y));
            Serial.print(",");
            Serial.println(int(tp_estimator.MagRaw.z));
    #endif
            serial_skip_count = 0;
        }
        serial_skip_count++;
#endif
	}
        
	return 0;
}
