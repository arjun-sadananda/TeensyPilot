#include <Arduino.h>
#include <Servo.h>
// #include <PWMServo.h>

// #include "teensyshot/ESCCMD.h"
// #include "teensyshot/DSHOT.h"


// FlexPWM4.0	22
// FlexPWM4.1	23	4.482 kHz
// FlexPWM4.2	2, 3
// FlexPWM2.3	36, 37

#define ESCPIN1 2
#define ESCPIN3 4

#define MODE_3D true
#define ESC_CALIB false

class TP_MOTOR_2DOF_SIMPLE{
protected:
    Servo ESC1, ESC3;
    uint16_t min_throttle = 1000, max_throttle = 2000, neutral_3d = 1460;
    uint16_t slope = 400;
    //deadband
    
    void ESC_setup(){

        ESC1.attach(ESCPIN1, min_throttle, max_throttle);
        ESC3.attach(ESCPIN3, min_throttle, max_throttle);

        ESC1.writeMicroseconds(neutral_3d);
        ESC3.writeMicroseconds(neutral_3d); // 3 beeps - powering on

        delay(1500);

        ESC1.writeMicroseconds((neutral_3d+max_throttle)/2);
        ESC3.writeMicroseconds((neutral_3d+max_throttle)/2);  // 1 long low beep - arming sequence begins

        delay(1500);
    #if ESC_CALIB
        // Calibration 
        // ESC1.writeMicroseconds(1110);
        // ESC1.writeMicroseconds(2230);
        // ESC1.writeMicroseconds(2000);
        // ESC2.writeMicroseconds(2000);
        // ESC3.writeMicroseconds(2000);
        // ESC4.writeMicroseconds(2000);   // single highs beeps - measure
        analogWrite(ESCPIN1, mid_ticks + (int) ticks_range/2.0 + 100);
        analogWrite(ESCPIN3, mid_ticks + (int) ticks_range/2.0 + 100);
        delay(7000);                   // rising beeps - calibration done
        analogWrite(ESCPIN1, mid_ticks);
        analogWrite(ESCPIN3, mid_ticks);
        // ESC1.writeMicroseconds(1000);
        // ESC2.writeMicroseconds(1000);
        // ESC3.writeMicroseconds(1000);
        // ESC4.writeMicroseconds(1000);   // double low beeps - measure
        delay(7000);                   // falling beeps - calibration done
        // Calibration Ended
    #endif
        ESC1.writeMicroseconds(neutral_3d);
        ESC3.writeMicroseconds(neutral_3d);   // 1 long high beep (if not callibrating) - arming sequence ends
        
        // set_motor_speeds(0,0);
        delay(5000);
    }
    // throttle percentage
    void set_motor_speeds(float m1, float m3){
#if !MODE_3D
        // add conditions
        // ESC1.writeMicroseconds(m1);
        // ESC2.writeMicroseconds(m2);
        // ESC3.writeMicroseconds(m3);
        // ESC4.writeMicroseconds(m4);
        if(m1>10.0)            m1=10.0;
        if(m3>20.0)            m3=20.0;
        if(m1<0)               m1=0;
        if(m3<0)               m3=0;
        analogWrite(ESCPIN1, min_ticks + (int) m1/100.0*ticks_range);
        analogWrite(ESCPIN3, min_ticks + (int) m3/100.0*ticks_range);

#else
        if(m1>30.0)            m1=30.0;
        if(m3>40.0)            m3=40.0;
        if(m1<-30.0)           m1=-30.0;
        if(m3<-40.0)           m3=-40.0;
        ESC1.writeMicroseconds(neutral_3d + m1/100.0*slope);
        ESC3.writeMicroseconds(neutral_3d + m3/100.0*slope);
#endif
        // IMPORTANT Add safety features, to prevent it from flying away
    }

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
};
