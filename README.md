# TeensyPilot

A bare-bones drone firmware primarily for Teensy4.0 or Teensy 4.1. 

This is work in progress and many important/usefull features are still being added.

Includes:
- TP_Math: Math Classes used in ArduPilot stripped down to the bare minimum needed for TeensyPilot. - Quaternion, Matrix3, Vector3, Vector2
- TP_Sense: Sensor Classes for I2C interface with MARG/MIMU sensors: MPU6050, QMC5883, LSM9DS1. And Barometer: BMP280. (SPI for LSM9DS1 in progress)
- TP_Motors: Functions for interfacing and calibrating a 4-in-1 BL-Heli-S ESC using PWM (Incomplete: DSHOT)
- TP_StateEstimate: TP_ESTIMATOR Class that inturn uses a variety of estimators (BKF-Basic KF- 2x1D KF, EKF(not working), TRIAD, MEKF, MEKF2, EMKF2(with TRAID Mode on)) for learning about attitude estimation.
  - The recommended Estimation Mode is: MEKF2_TRIAD
- TP_Control: in testing
  - Aim: To implement geometric attitude control to stabilise a drone under harsh initialization (throw mode). 
- TP_Display: Class for utilising the [Optimized ILI9341 library](https://github.com/PaulStoffregen/ILI9341_t3) for On-Board Firmware Debugging Display.
  - Display Modes (to be explained):
- CRSF interface: in development
 

