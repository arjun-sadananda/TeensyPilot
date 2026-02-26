#!/usr/bin/env python3

#
# https://github.com/crsf-wg/crsf/wiki
# This simple python parser implements a loose CRSF implementation. 
# Strict mode would require the packet start with CRSF_SYNC (0xC8) 
# but this ignores that byte and processes any packet that's 4-64 bytes long 
# and the CRC checks out. This is similar to the parsers in EdgeTX (2.9), iNav (7.0), and Betaflight (4.5).

# Note the default baud rate is arbitrarily chosen for testing. 
# Use 420k/416666 for receivers, or 400k+ etc for transmitters. 
# Half-duplex inverted (S.PORT / external module protocol) is not supported.

# Add the --tx option to also send CHANNELS_PACKED at 50Hz. The channel values will all be 1500us.

import time
import argparse
from enum import IntEnum

import math
import numpy as np

import serial
import pygame
import mss
import cv2

# from ultralytics import YOLO
# import torch

"""

Shared data

Frame
Anotation on?
Controller on?
"""

"""
# 
# CRSF 
#
# https://github.com/crsf-wg/crsf/wiki
# This simple python parser implements a loose CRSF implementation. 
# Strict mode would require the packet start with CRSF_SYNC (0xC8) 
# but this ignores that byte and processes any packet that's 
# 4-64 bytes long and the CRC checks out. This is similar to the parsers in EdgeTX (2.9), iNav (7.0), and Betaflight (4.5).
#
# This CRSF packet is sent to ELRS TX module (Radiomaster Nomad)
"""

CRSF_SYNC = 0xC8

class PacketsTypes(IntEnum):
    GPS = 0x02
    VARIO = 0x07
    BATTERY_SENSOR = 0x08
    BARO_ALT = 0x09
    HEARTBEAT = 0x0B
    VIDEO_TRANSMITTER = 0x0F
    LINK_STATISTICS = 0x14
    RC_CHANNELS_PACKED = 0x16
    ATTITUDE = 0x1E
    FLIGHT_MODE = 0x21
    DEVICE_INFO = 0x29
    CONFIG_READ = 0x2C
    CONFIG_WRITE = 0x2D
    RADIO_ID = 0x3A

def crc8_dvb_s2(crc, a) -> int:
  crc = crc ^ a
  for ii in range(8):
    if crc & 0x80:
      crc = (crc << 1) ^ 0xD5
    else:
      crc = crc << 1
  return crc & 0xFF

def crc8_data(data) -> int:
    crc = 0
    for a in data:
        crc = crc8_dvb_s2(crc, a)
    return crc

def crsf_validate_frame(frame) -> bool:
    return crc8_data(frame[2:-1]) == frame[-1]

def signed_byte(b):
    return b - 256 if b >= 128 else b

def packCrsfToBytes(channels) -> bytes:
    # channels is in CRSF format! (0-1984)
    # Values are packed little-endianish such that bits BA987654321 -> 87654321, 00000BA9
    # 11 bits per channel x 16 channels = 22 bytes
    if len(channels) != 16:
        raise ValueError('CRSF must have 16 channels')
    result = bytearray()
    destShift = 0
    newVal = 0
    for ch in channels:
        # Put the low bits in any remaining dest capacity
        newVal |= (ch << destShift) & 0xff
        result.append(newVal)

        # Shift the high bits down and place them into the next dest byte
        srcBitsLeft = 11 - 8 + destShift
        newVal = ch >> (11 - srcBitsLeft)
        # When there's at least a full byte remaining, consume that as well
        if srcBitsLeft >= 8:
            result.append(newVal & 0xff)
            newVal >>= 8
            srcBitsLeft -= 8

        # Next dest should be shifted up by the bits consumed
        destShift = srcBitsLeft

    return result

def channelsCrsfToChannelsPacket(channels) -> bytes:
    result = bytearray([CRSF_SYNC, 24, PacketsTypes.RC_CHANNELS_PACKED]) # 24 is packet length
    result += packCrsfToBytes(channels)
    result.append(crc8_data(result[2:]))
    return result

def handleCrsfPacket(ptype, data):
    if ptype == PacketsTypes.RADIO_ID and data[5] == 0x10:
        #print(f"OTX sync")
        pass
    elif ptype == PacketsTypes.LINK_STATISTICS:
        rssi1 = signed_byte(data[3])
        rssi2 = signed_byte(data[4])
        lq = data[5]
        snr = signed_byte(data[6])
        antenna = data[7]
        mode = data[8]
        power = data[9]
        # telemetry strength
        downlink_rssi = signed_byte(data[10])
        downlink_lq = data[11]
        downlink_snr = signed_byte(data[12])
        print(f"RSSI={rssi1}/{rssi2}dBm LQ={lq:03} mode={mode}") # ant={antenna} snr={snr} power={power} drssi={downlink_rssi} dlq={downlink_lq} dsnr={downlink_snr}")
    elif ptype == PacketsTypes.ATTITUDE:
        pitch = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10000.0
        roll = int.from_bytes(data[5:7], byteorder='big', signed=True) / 10000.0
        yaw = int.from_bytes(data[7:9], byteorder='big', signed=True) / 10000.0
        print(f"Attitude: Pitch={pitch:0.2f} Roll={roll:0.2f} Yaw={yaw:0.2f} (rad)")
    elif ptype == PacketsTypes.FLIGHT_MODE:
        packet = ''.join(map(chr, data[3:-2]))
        print(f"Flight Mode: {packet}")
    elif ptype == PacketsTypes.BATTERY_SENSOR:
        vbat = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10.0
        curr = int.from_bytes(data[5:7], byteorder='big', signed=True) / 10.0
        mah = data[7] << 16 | data[8] << 7 | data[9]
        pct = data[10]
        print(f"Battery: {vbat:0.2f}V {curr:0.1f}A {mah}mAh {pct}%")
    elif ptype == PacketsTypes.BARO_ALT:
        print(f"BaroAlt: ")
    elif ptype == PacketsTypes.DEVICE_INFO:
        packet = ' '.join(map(hex, data))
        print(f"Device Info: {packet}")
    elif data[2] == PacketsTypes.GPS:
        lat = int.from_bytes(data[3:7], byteorder='big', signed=True) / 1e7
        lon = int.from_bytes(data[7:11], byteorder='big', signed=True) / 1e7
        gspd = int.from_bytes(data[11:13], byteorder='big', signed=True) / 36.0
        hdg =  int.from_bytes(data[13:15], byteorder='big', signed=True) / 100.0
        alt = int.from_bytes(data[15:17], byteorder='big', signed=True) - 1000
        sats = data[17]
        print(f"GPS: Pos={lat} {lon} GSpd={gspd:0.1f}m/s Hdg={hdg:0.1f} Alt={alt}m Sats={sats}")
    elif ptype == PacketsTypes.VARIO:
        vspd = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10.0
        print(f"VSpd: {vspd:0.1f}m/s")
    elif ptype == PacketsTypes.RC_CHANNELS_PACKED:
        #print(f"Channels: (data)")
        pass
    else:
        packet = ' '.join(map(hex, data))
        print(f"Unknown 0x{ptype:02x}: {packet}")


# CRSF Serial Setup

PORT = 'COM6'
BAUD = 921600
TX_ENABLED = True
CRSF_initialized = False
try:
    ser = serial.Serial(port=PORT, baudrate=BAUD, timeout=2)
    CRSF_initialized = True
except Exception as e:
    print(f"Serial Error: {e}")
    CRSF_initialized = False
# help='Enable sending CHANNELS_PACKED every 20ms (all channels 1500us)'

""" 
#
#  Joystick Input Setup
#
#  This uses pygame to read the EdgeTX radio in USB Joystick mode.
#  
#  4 channels for the 4 sticks - forwarded to TX in manual mode and 
#  1 button                    - for the AUTO mode toggle.
#
"""

JOY_DEVICE_ID = 0  # EdgeTX radio joystick ID
radio_initialized = False
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("No EdgeTX radio found. Ensure it is in USB Joystick mode.")
    # exit()
else:
    radio_initialized = True
    radio = pygame.joystick.Joystick(JOY_DEVICE_ID)
    radio.init()
    print(f"Connected to: {radio.get_name()}")

"""
# 
#  Frame Input Setup
# 
"""

# 1200 TVL TV Lines
# 1280x960
# 640x480
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

SCREEN_ID = None       # Set to None to use camera instead of screen capture
CAMERA_ID = 1    # Set to None to use screen capture instead of camera
# CAMERA / SCREEN
if CAMERA_ID is not None:
    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")
elif SCREEN_ID is not None:
    sct = mss.mss()

frame = None
annotated_frame = None

"""###############################
# 
#  Vision Mode
# 
###############################"""

OptFlowMode = True
YOLOMode = False
YOLO_OptFlowMode = False

"""
# 
# Optical Flow Point Tracking 
# 
"""
point_selected = False
p0       = None
old_gray = None
selected_point = None
# Parameters for Lucas-Kanade optical flow
lk_params = dict(winSize=(15, 15),
                maxLevel=2,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

def select_point(event, x, y, flags, param):
    global p0, point_selected, selected_point
    if event == cv2.EVENT_LBUTTONDOWN:
        # Store the clicked point as a NumPy array of float32,
        # formatted correctly for calcOpticalFlowPyrLK
        p0 = np.array([[[x, y]]], dtype=np.float32)
        point_selected = True
        selected_point = [x, y]
        print(f"Tracking point selected at: ({x}, {y})")

def calcOptFlow(frame):
    global p0, old_gray
    target_point = -1, 20

    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if old_gray is not None and point_selected:
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

        if p1 is not None and st.any():
            good_new = p1[st == 1]
            good_old = p0[st == 1]

            a, b = good_new.ravel()
            c, d = good_old.ravel()
            target_point = [int(a), int(b)]

            # Update the previous points
            p0 = good_new.reshape(-1, 1, 2)

    # Update the previous frame
    old_gray = frame_gray.copy()

    return target_point

"""
#
# Create a window and bind the mouse callback function to it
#
"""
cv2.namedWindow('Point Tracking', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Point Tracking', int(FRAME_WIDTH), int(FRAME_HEIGHT))
cv2.setMouseCallback('Point Tracking', select_point)


"""
# 
# YOLO ID and Tracking
# 
"""

# MODEL_PATH = r"C:\Users\Arjun\Documents\GitHub\TeensyPilot\include\TP_VisionControl\best.pt"
# CONF_THRES = 0.25
# YOLO_DEVICE = 0 if torch.cuda.is_available() else "cpu"
# # ---------------- LOAD MODEL ----------------
# model = YOLO(MODEL_PATH)
# model.to(YOLO_DEVICE)

# def get_YOLO_centroid(frame):

#     results = model.predict(
#         source=frame,
#         task="obb",
#         conf=CONF_THRES,
#         device=YOLO_DEVICE,
#         verbose=False,
#         # imgsz=640,
#         tracker="bytetrack.yaml"
#     )
#     # ----- Extract OBB points -----
#     if hasattr(results[0], "obb") and results[0].obb is not None:
#         # shape: (N, 4, 2)
#         # obb_points = results[0].obb.xywhr.cpu().numpy()
#         obb_points = results[0].obb.xyxyxyxy.cpu().numpy()
#         for i, pts in enumerate(obb_points):
#             # pts = [[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
#             x = pts[:, 0]
#             y = pts[:, 1]

#             # -------- CENTROID --------
#             cx = int(x.mean())
#             cy = int(y.mean())
#             # if i==0:
#             return cx, cy
    
#     return -1, -1
    

"""
# 
# Controller
# 
# Control parameters and Function
#
"""

controller_on = False

cx = 0
cy = 0
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
CAMERA_ANGLE = 25.0  # degrees
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


"""
# Main Loop
# 1. Read Joystick
# 2. Get Frame
# 3. If AUTO mode is on, get target and compute control
# 4. Send CHANNELS_PACKED
"""

# time_passed = time.time()
# start_time = time.time()
# time_period = 2.0 # seconds for full sweep

LOOP_RATE =  0.01 # 100Hz?
AUTO_Switch = 0
Vision_Switch = 1
roll, pitch, throttle, yaw_rate = 0, 0, 0, 0

while True:
    start_time = time.perf_counter()
    # ----- Pygame Event Pump -----
    pygame.event.pump()
    if radio_initialized:
        # Read Axes (Sticks: Aileron, Elevator, Throttle, Rudder)
        # Values typically range from -1.0 to 1.0
        axes = [radio.get_axis(i) for i in range(radio.get_numaxes())]

        roll     = int(axes[0]*RC_STICK_RANGE/2) + RC_STICK_MID
        pitch    = int(axes[1]*RC_STICK_RANGE/2) + RC_STICK_MID
        throttle = int(axes[2]*RC_STICK_RANGE/2) + RC_STICK_MID
        yaw_rate = int(axes[3]*RC_STICK_RANGE/2) + RC_STICK_MID

        # Read Buttons (Switches mapped as buttons)
        buttons = [radio.get_button(i) for i in range(radio.get_numbuttons())]
        AUTO_Switch = buttons[2]
        Vision_Switch = buttons[0]
    
    # --------- Get Frame ---------
    if CAMERA_ID:
        ret, frame = cap.read()
        # height, width, channels = frame.shape
        # print(f"Width: {width}, Height: {height}, Channels: {channels}")

        if not ret:
            break
    elif SCREEN_ID:
        sct_img = sct.grab(sct.monitors[SCREEN_ID])
        frame = np.array(sct_img)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT), cv2.INTER_AREA)
    # ----- If AUTO mode is on -----
    if Vision_Switch>0:
        # -------- Get target ---------
        if OptFlowMode:
            cx, cy = calcOptFlow(frame)
        elif YOLOMode:
            pass
            # cx, cy = get_YOLO_centroid(frame)
        elif YOLO_OptFlowMode:
            cx, cy = -1, -1

        if AUTO_Switch>0:
            controller_on = True
            
            if cx != -1 and cy != -1:
                e_x = cx - FRAME_WIDTH / 2
                e_x /= (FRAME_WIDTH / 2)
                e_y = cy - FRAME_HEIGHT / 2
                e_y /= (FRAME_HEIGHT / 2)
                roll, pitch, throttle, yaw_rate = controller(e_x, e_y)
        else:
            controller_on = False
            # with vis_con_data["lock"]:
            #     vis_con_data["controller_on"] = False


    # ----- Send CHANNELS_PACKED -----
    if CRSF_initialized:
        ser.write(channelsCrsfToChannelsPacket([roll, pitch, throttle, yaw_rate] + [992 for ch in range(12)]))
    

    ####
    # Display stuff

    # Vision Mode
    # Control Mode
    # Target box/point
    # YOLO stuff

    TextHeight = int(FRAME_HEIGHT/30)
    fontFace = cv2.FONT_HERSHEY_SIMPLEX
    thickness = 2
    fontScale = cv2.getFontScaleFromHeight(fontFace, TextHeight, thickness)
    TextBox_x = int(FRAME_HEIGHT/20)
    TextBox_y = int(FRAME_HEIGHT/20)
    if controller_on:
        cv2.putText(frame, "AUTO", (TextBox_x, TextBox_y), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), thickness)
    else:
        cv2.putText(frame, "MANUAL", (TextBox_x, TextBox_y), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 0, 255), thickness)

    if Vision_Switch>0:
        cv2.putText(frame, "Vision ON", (TextBox_x, int(TextBox_y + 1.3*TextHeight)), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), thickness)
    else:
        cv2.putText(frame, "Vision OFF", (TextBox_x, int(TextBox_y + 1.3*TextHeight)), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), thickness)

    # joy_center_l = (int(FRAME_WIDTH/2)-55, int(FRAME_HEIGHT/2))
    # joy_center_r = (int(FRAME_WIDTH/2)+55, int(FRAME_HEIGHT/2))

    H_CENTER = FRAME_WIDTH*1.5/5
    V_CENTER = FRAME_HEIGHT*5/6

    SIZE_HALF = FRAME_HEIGHT/10
    SIZE = SIZE_HALF*2
    H_SPACE = SIZE*.6

    cv2.rectangle(frame, (int(H_CENTER-H_SPACE-SIZE_HALF), int(V_CENTER-SIZE_HALF)), (int(H_CENTER-H_SPACE+SIZE_HALF), int(V_CENTER+SIZE_HALF)), (200, 200, 200), 2) # center point
    cv2.rectangle(frame, (int(H_CENTER+H_SPACE-SIZE_HALF), int(V_CENTER-SIZE_HALF)), (int(H_CENTER+H_SPACE+SIZE_HALF), int(V_CENTER+SIZE_HALF)), (200, 200, 200), 2) # center point
    
    cv2.circle(frame, (int(H_CENTER-H_SPACE),                                              int(V_CENTER)                           ),5, (150, 150, 150), -1) # center point
    cv2.circle(frame, (int(H_CENTER-H_SPACE+SIZE*(yaw_rate-RC_STICK_MID)/RC_STICK_RANGE),  int(V_CENTER-SIZE*(throttle-RC_STICK_MID)/RC_STICK_RANGE)),5, (255, 0, 0), -1) # center point
    cv2.circle(frame, (int(H_CENTER+H_SPACE),                                              int(V_CENTER)                           ),5, (150, 150, 150), -1) # center point
    cv2.circle(frame, (int(H_CENTER+H_SPACE+SIZE*(roll-RC_STICK_MID)/RC_STICK_RANGE),      int(V_CENTER-SIZE*(pitch-RC_STICK_MID)/RC_STICK_RANGE)   ),5, (255, 0, 0), -1) # center point
    
    if point_selected:
        cv2.circle(frame, (int(selected_point[0]), int(selected_point[1])), 5, (150, 150, 150), -1) # center point
        cv2.circle(frame, (int(cx), int(cy)), 5, (255, 0, 0), -1) # center point

    elapsed = time.perf_counter() - start_time
    cv2.putText(frame, f"Loop Time: {elapsed*1000:2.0f}ms", (TextBox_x, int(TextBox_y+2.6*TextHeight)), fontFace, fontScale, (255, 255, 0), thickness)
    
    cv2.imshow('Point Tracking', frame)

    k = cv2.waitKey(30) & 0xff # used to be 30
    if k == ord('q'):  # Press 'Esc' to exit
        break
    elif k == ord('c'): # Press 'c' to reset and select a new point
        point_selected = False
        p0 = None


    
    # time.sleep(max(0, LOOP_RATE - elapsed))

cv2.destroyAllWindows()

