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


#define ESC_CALIB false

class TP_MOTOR_2DOF{
protected:
    uint8_t resolution;
    float frequency;
    float total_us, min_us;
    uint16_t min_ticks, total_ticks, ticks_range;
    // Servo ESC1, ESC2, ESC3, ESC4;
    
    void ESC_setup(){
        // 0 - 32757  -> 15     4577.64 Hz
        // 0 - 16383  -> 14     9155.27 Hz
        // 0 - 8191   -> 13     18310.55 Hz
        // 0 - 4095   -> 12     36621.09 Hz
        // 0 - 2047   -> 11     73242.19 Hz

        // multishot  --- min_ticks = 1874.99, max_ticks 3749.9, tot: 4095
        // frequency   = 30000;
        // resolution  = 12;
        // min_us      = 12.5;

        //  oneshot42
        // frequency   = 9155.27;    // must be less that 12195 for oneshot42
        // resolution  = 14;
        // min_us      = 42.0;

        //  oneshot125
        // frequency   = 1000.0;    // must be less that 12195 for oneshot42
        // resolution  = 15;
        // min_us      = 125.0;

        // standard pwm  --- min_ticks: 16056.32   max 32112.64   tot 32757
        frequency   = 490.0;
        resolution  = 15;
        min_us      = 1000.0;

        total_ticks = pow(2,resolution);
        total_us    = 1e6/frequency;
        min_ticks   = min_us/total_us*total_ticks;
        ticks_range = min_ticks;

	    digitalWrite(ESCPIN1, LOW);
	    digitalWrite(ESCPIN3, LOW);

        pinMode(ESCPIN1, OUTPUT);
        pinMode(ESCPIN3, OUTPUT);

        // 0us    42us       84us       100us       126us
        // 0      3440.22    6880.44    8191
        // 0      1720.32    3440.64    4096

        // analogWriteFrequency(pin, 50);

        // 146484.38
        // 100us        -> 10000   Hz    
        // 109.2267us   ->  9155.27Hz     14bit
        // 126us        ->  7936.5 Hz    
        analogWriteFrequency(ESCPIN1, frequency);
        analogWriteFrequency(ESCPIN3, frequency);

        analogWriteResolution(resolution);

        // ESC1.attach(ESCPIN1, 1000, 2000);
        // ESC2.attach(ESCPIN2, 1000, 2000);
        // ESC3.attach(ESCPIN3, 1000, 2000);
        // ESC4.attach(ESCPIN4, 1000, 2000);

        // ESC1.writeMicroseconds(1000);
        // ESC2.writeMicroseconds(1000);
        // ESC3.writeMicroseconds(1000);
        // ESC4.writeMicroseconds(1000); // 3 beeps - powering on
        set_motor_speeds(0,0);
        delay(1500);

        // ESC1.writeMicroseconds(1200);
        // ESC2.writeMicroseconds(1200);
        // ESC3.writeMicroseconds(1200);
        // ESC4.writeMicroseconds(1200);   // 1 long low beep - arming sequence begins
        set_motor_speeds(20,20);
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
        // ESC1.writeMicroseconds(1000);
        // ESC2.writeMicroseconds(1000);
        // ESC3.writeMicroseconds(1000);
        // ESC4.writeMicroseconds(1000);   // 1 long high beep (if not callibrating) - arming sequence ends
        set_motor_speeds(0,0);
        delay(5000);
    }
    // throttle percentage
    void set_motor_speeds(float m1, float m3){
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
