"""
# 
# Controller
# Control parameters and Function
#
"""
import time
import math


e_x_prev = 0.0
e_y_prev = 0.0
prev_time = time.time()

# 1811+172 = 1639/2 = 992
# 1811-992 = 819
RC_STICK_MIN = 172
RC_STICK_MAX = 1811
RC_STICK_RANGE = RC_STICK_MAX - RC_STICK_MIN
RC_STICK_MID = (RC_STICK_MIN + RC_STICK_MAX) // 2

ANGLE_LIMIT = 55.0  # degrees
CAMERA_ANGLE = 20.0  # degrees
# 848, 678
# Ideally Use LOS instead of pixel errors directly
e_y_integral = 0.0

HOVER_THROTTLE_PC = 0.25  # Portion of throttle range to hover at level flight

# returns u_roll, u_pitch, u_throttle, u_yaw_rate
def controller(e_x, e_y):
    global e_x_prev, e_y_prev, prev_time, e_y_integral

    dt = time.time() - prev_time  # Time step, adjust as needed
    prev_time = time.time()

    d_e_x = (e_x - e_x_prev)/dt
    d_e_y = (e_y - e_y_prev)/dt

    e_x_prev = e_x
    e_y_prev = e_y

    if abs(e_y) < .25:
        e_y_integral += e_y * dt  # Simple integral term, could be accumulated over time
    else:
        e_y_integral = 0.0  # Reset integral if error is large
    Kp_roll     = RC_STICK_RANGE*.3
    Kp_yaw_rate = RC_STICK_RANGE*.3
    Kp_pitch    = RC_STICK_RANGE*.3
    Kp_throttle = RC_STICK_RANGE*2
    Kd_roll     = 0.0
    Kd_yaw_rate = 0.0
    Kd_pitch    = 0.0
    Kd_throttle = 0.0
    Ki_throttle = RC_STICK_RANGE*1
    # convert e_y to angle (use LOS)

    PITCH_BIAS = RC_STICK_RANGE/2 * (CAMERA_ANGLE / ANGLE_LIMIT)

    # Total control outputs
    u_roll     = RC_STICK_MID + Kp_roll * e_x     + Kd_roll * d_e_x
    u_yaw_rate = RC_STICK_MID + Kp_yaw_rate * e_x + Kd_yaw_rate * d_e_x
    u_pitch    = RC_STICK_MID + Kp_pitch * e_y    + Kd_pitch * d_e_y    + PITCH_BIAS

    # replace pitch with tilt angle for throttle feedforward
    pitch = ANGLE_LIMIT * (u_pitch - RC_STICK_MID) / (RC_STICK_RANGE/2)
    throttle_ff = RC_STICK_MIN + (RC_STICK_RANGE * HOVER_THROTTLE_PC / math.cos(math.radians(pitch)))  # Adjust as needed
    u_throttle  = throttle_ff - Kp_throttle * e_y   + Kd_throttle * d_e_y    #- Ki_throttle * e_y_integral
    
    # Clip to valid range
    u_roll      = max(RC_STICK_MIN, min(RC_STICK_MAX, int(u_roll)))
    u_yaw_rate  = max(RC_STICK_MIN, min(RC_STICK_MAX, int(u_yaw_rate)))
    u_pitch     = max(RC_STICK_MIN, min(RC_STICK_MAX, int(u_pitch)))
    u_throttle  = max(RC_STICK_MIN, min(RC_STICK_MAX, int(u_throttle)))

    return u_roll, u_pitch, u_throttle, u_yaw_rate
