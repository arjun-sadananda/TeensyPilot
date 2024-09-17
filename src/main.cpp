// #include <zephyr.h>
#include <Arduino.h>

#include "TP_Display.h"
#include "TP_Serial.h"
// #include "TP_SD.h"
#include "TP_StateEstimate\TP_ESTIMATOR.h"
#include "TP_Control\TP_Control.h"

#include "TP_RC.h"


#include "SD.h"

#define SD_LOG_OFF 0
#define SD_LOG_ESTIMATOR 1
#define SD_LOG_ALL_ESTIMATOR 2
#define SD_LOG_DRONE_MONITOR 3
#define SD_LOG_TWO_ESTIMATORS 4

// #define DEBUG_MODE true
// #define DEBUG_AHRS true
// #define DEBUG_ALT_EST true

// #define MOTORS_ON false

#define display_type    DRONE_MONITOR
#define serial_type     SERIAL_DEBUG
#define sd_log_type     SD_LOG_TWO_ESTIMATORS

#if serial_type == SERIAL_DEBUG
    #define debug(x) Serial.print(x); Serial.print(" ");
    #define debugln(x) Serial.println(x)
#else
    #define debug(x)
    #define debugln(x)
#endif

uint32_t loop_start_time, iter_start_time, control_start_time;
uint32_t read_time, estimate_time, display_time;//, control_time;

// bool ready = false;
bool controller_on = false;
// const int throw_delay = 200;
bool prev_arm_status = false;

TP_ESTIMATOR tp_estimator;
TP_Control tp_control;

#if display_type != DISPLAY_OFF
    TP_Display tp_display;
    #define displayStatus(x) tp_display.printStatus(x);
#else
    #define displayStatus(x)
#endif

#if sd_log_type != SD_LOG_OFF
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

    // tp_rc.init();


    debugln("Sensor Initialising");
    displayStatus("Sensor Initialising");
    tp_estimator.init_sensors();
    debugln("Sensor Calibration");
    displayStatus("Sensor Calibration");
    delay(1000);
    displayStatus("Recording...");
    tp_estimator.calibrate_sensors();

    debugln("Estimator Initialising");
    displayStatus("Estimator Initialising");
    tp_estimator.init_estimator();

#if sd_log_type != SD_LOG_OFF
    if(!SD.begin(BUILTIN_SDCARD)){
        displayStatus("SD intialisation failed!");
    }
    else{
        displayStatus("SD intialised!");
    }
    delay(1000);
    
    myFile = SD.open("log.csv", FILE_WRITE);
    if (myFile) {
        displayStatus("File opened!");
        // String data = "time,q0,q1,q2,q3,wx,wy,wz,thrust,qd1,qd2,qd3,qd4,f,Mx,My,Mz";
        // SD_LOG_TWO_ESTIMATORS
        String data = "time,wx,wy,wz,ax,ay,az,mx,my,mz,q0,q1,q2,q3,q0,q1,q2,q3,q0,q1,q2,q3,w_e,a_e,m_e,e_t,e_m2,e_m2t";
        // SD_get_header(data, sd_log_type);
        
        myFile.print(data);
        data = String(    "," + String(tp_estimator.a_ref.x, 6) 
                        + "," + String(tp_estimator.a_ref.y, 6) 
                        + "," + String(tp_estimator.a_ref.z, 6)
                        + "," + String(tp_estimator.m_ref.x, 6)
                        + "," + String(tp_estimator.m_ref.y, 6)
                        + "," + String(tp_estimator.m_ref.z, 6));
        myFile.println(data);

        delay(1000);
    }
#endif
    debug(tp_estimator.a_ref.x);
    debug(tp_estimator.a_ref.y);
    debugln(tp_estimator.a_ref.z);
    debug(tp_estimator.m_ref.x);
    debug(tp_estimator.m_ref.y);
    debugln(tp_estimator.m_ref.z);

    debugln("ESC Initialising");
    displayStatus("ESC Initialising");
    tp_control.init_motors();

    // tp_control.motor_test();

    // control_start_time = micros();
    debugln("Main Loop Running");
    displayStatus("Main Loop Running");


    loop_start_time = micros();
    /*
     * Main Loop
     *
     */

	while(1){
		iter_start_time = micros();

        /*
         * Read Sensors
         */
        tp_estimator.read_sensors();
        read_time = micros() - iter_start_time; 

        /*
         * Run Attitude Estimator
         */
        tp_estimator.estimate_attitude();
        estimate_time = micros() - iter_start_time - read_time;

        /*
         * Update RC Commands
         */
        // tp_rc.update();


        /*
         * Step RPM
         */
        // static uint32_t rec_stop_time = 15000000;
        // if(micros()-loop_start_time < 3000000)
        //     tp_control.run_all_motors(0.0);
        // else if(micros()-loop_start_time < 6000000)
        //     tp_control.run_all_motors(10.0);
        // else if(micros()-loop_start_time < 9000000)
        //     tp_control.run_all_motors(40.0);
        // else if(micros()-loop_start_time < 12000000)
        //     tp_control.run_all_motors(75.0);

        static uint32_t rec_stop_time = 24000000;
        if(micros()-loop_start_time < 3000000)
            tp_control.run_all_motors(0.0);
        else if(micros()-loop_start_time < 6000000)
            tp_control.run_all_motors(5.0);
        else if(micros()-loop_start_time < 9000000)
            tp_control.run_all_motors(10.0);
        else if(micros()-loop_start_time < 12000000)
            tp_control.run_all_motors(20.0);
        else if(micros()-loop_start_time < 15000000)
            tp_control.run_all_motors(30.0);
        else if(micros()-loop_start_time < 18000000)
            tp_control.run_all_motors(40.0);
        else if(micros()-loop_start_time < 21000000)
            tp_control.run_all_motors(70.0);
        else
            tp_control.stop_motors();

        /*
         * Controller Switch: Throw Trigger
         */

        // if (digitalRead(8) == LOW){  // Button Pressed
        //     ready = true;
        //     controller_on = false;
        // }
        // if (ready == true && digitalRead(8) == HIGH){   // Button Released
        //     ready = false;
        //     displayStatus("Controller Running");
        //     if (!controller_on)
        //         delay(throw_delay);
        //     control_start_time = micros();
        //     tp_control.clear_integral();
        //     controller_on = true;
        // }

        // if (tp_rc.rc_command.arm == false)
        //     controller_on = false;

        // if (prev_arm_status == true && tp_rc.rc_command.arm == false){
        //     displayStatus("Disarmed");
        //     prev_arm_status = false;
        //     controller_on = false;
        // }
        // if (prev_arm_status == false && tp_rc.rc_command.arm == true){
        //     displayStatus("Armed");
        //     tp_control.clear_integral();
        //     prev_arm_status = true;
        //     controller_on = true;
        // }

        
        // M2 = euler_attitude_control( R, tp_estimator.omega);
        
        // if(int((micros() - control_start_time)/1000000.0) < 6 && controller_on == true){ // Button Not Pressed
        //     tp_control.throw_mode_controller(tp_estimator.q, tp_estimator.omega, true);
        // }
        // else{
        //     controller_on = false;
        //     tp_control.throw_mode_controller(tp_estimator.q, tp_estimator.omega, false);
        //     tp_control.stop_motors();
        //     // displayStatus("Controller Stopped");
        // }
        // control_time = micros() - iter_start_time - read_time - estimate_time;

        // const static float max_rp = .75, max_yaw = 1;
        // static float thrust, roll, pitch, yaw; 
        // thrust = (tp_rc.rc_command.throttle-1000)/500.0;
        // roll = -(tp_rc.rc_command.roll-1500)/500.0*max_rp;
        // pitch = (tp_rc.rc_command.pitch-1500)/500.0*max_rp;
        // yaw = (tp_rc.rc_command.yaw-1500)/500.0*max_yaw;

        // static Matrix3f Rd;
        // Rd.from_euler(roll, pitch, yaw);

        // if (controller_on){
        //     tp_control.angle_mode_controller(tp_estimator.q, tp_estimator.omega, thrust, Rd, true);
        // }
        // else{
        //     tp_control.angle_mode_controller(tp_estimator.q, tp_estimator.omega, thrust, Rd, false);
        //     tp_control.stop_motors();
        // }

#if sd_log_type != SD_LOG_OFF
    #if sd_log_type == SD_LOG_ALL_ESTIMATORS
        static int sd_log_count = 0;
        static String data;
        if(sd_log_count < 500 && myFile){
            data = String(iter_start_time)
                        + "," + String(tp_estimator.GyroRate.x, 6) 
                        + "," + String(tp_estimator.GyroRate.y, 6) 
                        + "," + String(tp_estimator.GyroRate.z, 6)
                        + "," + String(tp_estimator.UnitAccVect.x, 6) 
                        + "," + String(tp_estimator.UnitAccVect.y, 6) 
                        + "," + String(tp_estimator.UnitAccVect.z, 6)
                        + "," + String(tp_estimator.UnitMagVect.x, 6) 
                        + "," + String(tp_estimator.UnitMagVect.y, 6) 
                        + "," + String(tp_estimator.UnitMagVect.z, 6);
            // for(int i =0; i<36; i++)
            //         data.append("," + String(tp_estimator.tp_mekf2.P[i] , 6));
            // for(int i = 0; i<6; i++)
            //     for(int j =0; j<9; j++)
            //         data.append("," + String(tp_estimator.tp_mekf2.K[i*9+j], 6));

            // for(int i =0; i<36; i++)
            //         data.append("," + String(tp_estimator.tp_mekf2_acc.P[i] , 6));
            // for(int i = 0; i<6; i++)
            //     for(int j =0; j<9; j++)
            //         data.append("," + String(tp_estimator.tp_mekf2_acc.K[i*9+j], 6));

            for(int i =0; i<36; i++)
                    data.append("," + String(tp_estimator.tp_mekf2_triad.P[i] , 6));
            for(int i = 0; i<6; i++)
                for(int j =0; j<9; j++)
                    data.append("," + String(tp_estimator.tp_mekf2_triad.K[i*9+j], 6));

            myFile.println(data);
            sd_log_count ++;
        }
        if(sd_log_count == 500){
            myFile.close();
            tp_display.printStatus("Done Logging");
            sd_log_count ++;
        }
    #elif sd_log_type == SD_LOG_DRONE_MONITOR
        static int sd_log_count = 0;
        static bool log_on = false;
        static String data;
        if(!log_on && controller_on)
            log_on = true;
        if(log_on && !controller_on){
            log_on = false;
            myFile.close();
            tp_display.printStatus("Done Logging");
            delay(1000);
        }
        if(log_on && myFile && sd_log_count>10){
            static Quaternion qd;
            qd.from_rotation_matrix(Rd);
            data = String(iter_start_time)
                    + "," + String(tp_estimator.q.q1, 6)
                    + "," + String(tp_estimator.q.q2, 6)
                    + "," + String(tp_estimator.q.q3, 6)
                    + "," + String(tp_estimator.q.q4, 6)
                    + "," + String(tp_estimator.omega.x, 6)
                    + "," + String(tp_estimator.omega.y, 6)
                    + "," + String(tp_estimator.omega.z, 6)

                    + "," + String(thrust, 6)
                    + "," + String(qd.q1, 6)
                    + "," + String(qd.q2, 6)
                    + "," + String(qd.q3, 6)
                    + "," + String(qd.q4, 6)
                    + "," + String(tp_control.f, 6)
                    + "," + String(tp_control.M.x, 6)
                    + "," + String(tp_control.M.y, 6)
                    + "," + String(tp_control.M.z, 6);
            myFile.println(data);
            sd_log_count = 0;
        }
        sd_log_count++;
    #elif sd_log_type == SD_LOG_TWO_ESTIMATORS
        static int sd_log_count = 0;
        static String data;
        static Quaternion q_no_triad, q_only_triad, identity;
        //sd_log_count < 500
        if(micros()-loop_start_time < rec_stop_time && myFile){
            q_no_triad = tp_estimator.tp_mekf2.get_q();
            q_only_triad.from_rotation_matrix(tp_estimator.tp_triad.DCM);
            data = String(iter_start_time-loop_start_time)
                        + "," + String(tp_estimator.GyroRate.x, 6) 
                        + "," + String(tp_estimator.GyroRate.y, 6) 
                        + "," + String(tp_estimator.GyroRate.z, 6)
                        + "," + String(tp_estimator.UnitAccVect.x, 6) 
                        + "," + String(tp_estimator.UnitAccVect.y, 6) 
                        + "," + String(tp_estimator.UnitAccVect.z, 6)
                        + "," + String(tp_estimator.UnitMagVect.x, 6) 
                        + "," + String(tp_estimator.UnitMagVect.y, 6) 
                        + "," + String(tp_estimator.UnitMagVect.z, 6)

                        + "," + String(q_only_triad.q1, 6)
                        + "," + String(q_only_triad.q2, 6)
                        + "," + String(q_only_triad.q3, 6)
                        + "," + String(q_only_triad.q4, 6)
                        + "," + String(q_no_triad.q1, 6)
                        + "," + String(q_no_triad.q2, 6)
                        + "," + String(q_no_triad.q3, 6)
                        + "," + String(q_no_triad.q4, 6)
                        + "," + String(tp_estimator.q.q1, 6)
                        + "," + String(tp_estimator.q.q2, 6)
                        + "," + String(tp_estimator.q.q3, 6)
                        + "," + String(tp_estimator.q.q4, 6)

                        + "," + String(tp_estimator.GyroRate.length(), 6)
                        + "," + String(acos(tp_estimator.UnitAccVect.dot(tp_estimator.a_ref)), 6)
                        + "," + String(acos(tp_estimator.UnitMagVect.dot(tp_estimator.m_ref)), 6)

                        + "," + String(q_only_triad.angular_difference2(identity),6)
                        + "," + String(q_no_triad.angular_difference2(identity),6)
                        + "," + String(tp_estimator.q.angular_difference2(identity),6);

            myFile.println(data);
            // sd_log_count ++;
        }
        else if(sd_log_count == 0){
            myFile.close();
            displayStatus("Done Logging");
            sd_log_count ++;
        }
    #endif
#endif

#if display_type != DISPLAY_OFF
        static int display_skip_count = 0;
        if(display_skip_count > 50 && !controller_on){
            // tp_rc.update();
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

            tp_display.printVector(tp_estimator.UnitAccVect, 150, 70);
            tp_display.printVector(tp_estimator.UnitMagVect, 150, 110);

            // digitalWrite(LSM_CSAG_pin, LOW);
            // SPI1.transfer(0x0F | 0x80);// | 0x40);//| 0x40 LIS3MDL_REG_OUT_X_L WHO_AM_I: 104 = 01101000
            // tp_display.printStatus(SPI1.transfer(0), 100);
            // digitalWrite(LSM_CSAG_pin, HIGH);
            
            // digitalWrite(LSM_CSM_pin, LOW);
            // SPI1.transfer(LIS3MDL_REG_WHO_AM_I | 0x80);// | 0x40);//| 0x40 LIS3MDL_REG_OUT_X_L WHO_AM_I: 104 = 01101000 = 61
            // tp_display.printStatus(SPI1.transfer(0), 130);
            // digitalWrite(LSM_CSM_pin, HIGH);

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
            // tp_display.displayVel(tp_control.v, tp_control.get_max_vel());
            static int rc_comm[4];
            rc_comm[0] = tp_rc.rc_command.roll;
            rc_comm[1] = tp_rc.rc_command.pitch;
            rc_comm[2] = tp_rc.rc_command.yaw;
            rc_comm[3] = tp_rc.rc_command.throttle;

            tp_display.displayVel(rc_comm, 1000);

            /*
             * Display time for different processes
             * Sensor Reading, Estimator dt value, display time
             */
            display_time = micros() - iter_start_time - read_time - estimate_time;// - control_time;
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
            tp_display.printTime((micros()-iter_start_time));
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
    // debug(tp_estimator.q.q1);
    // debug(tp_estimator.q.q2);
    // debug(tp_estimator.q.q3);
    // debugln(tp_estimator.q.q4);
#endif
	}
	return 0;
}



