#include <Arduino.h>
#include <Servo.h>

// #include "teensyshot/ESCCMD.h"
// #include "teensyshot/DSHOT.h"


// FlexPWM4.0	22
// FlexPWM4.1	23	4.482 kHz
// FlexPWM4.2	2, 3
#define ESCPIN1 2
#define ESCPIN2 3
#define ESCPIN3 4
#define ESCPIN4 5


#define ESC_CALIB false

class TP_MOTOR{
protected:
    Servo ESC1, ESC2, ESC3, ESC4;

    void ESC_setup(){
        ESC1.attach(ESCPIN1, 1000, 2000);
        ESC2.attach(ESCPIN2, 1000, 2000);
        ESC3.attach(ESCPIN3, 1000, 2000);
        ESC4.attach(ESCPIN4, 1000, 2000);

        ESC1.writeMicroseconds(1000);
        ESC2.writeMicroseconds(1000);
        ESC3.writeMicroseconds(1000);
        ESC4.writeMicroseconds(1000); // 3 beeps - powering on
        delay(1500);

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
    }

    void set_motor_speeds(int m1, int m2, int m3, int m4){
        // add conditions
        ESC1.writeMicroseconds(m1);
        ESC2.writeMicroseconds(m2);
        ESC3.writeMicroseconds(m3);
        ESC4.writeMicroseconds(m4);

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
