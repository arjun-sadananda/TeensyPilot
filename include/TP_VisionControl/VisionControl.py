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

from ultralytics import YOLO
import torch

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

PORT = 'COM19'
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

pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("No EdgeTX radio found. Ensure it is in USB Joystick mode.")
    exit()
radio = pygame.joystick.Joystick(JOY_DEVICE_ID)
radio.init()
print(f"Connected to: {radio.get_name()}")

"""
# 
#  Frame Input Setup
# 
"""

FRAME_WIDTH = 1920
FRAME_HEIGHT = 1080

SCREEN_ID = 2       # Set to None to use camera instead of screen capture
CAMERA_ID = None    # Set to None to use screen capture instead of camera
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
# Parameters for Lucas-Kanade optical flow
lk_params = dict(winSize=(15, 15),
                maxLevel=2,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

def select_point(event, x, y, flags, param):
    global p0, point_selected
    if event == cv2.EVENT_LBUTTONDOWN:
        # Store the clicked point as a NumPy array of float32,
        # formatted correctly for calcOpticalFlowPyrLK
        p0 = np.array([[[x, y]]], dtype=np.float32)
        point_selected = True
        print(f"Tracking point selected at: ({x}, {y})")

def calcOptFlow(frame):
    global p0, old_gray
    target_point = -1, -1

    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if old_gray is not None and p0 is not None:
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

MODEL_PATH = r"C:\Users\Arjun\Documents\GitHub\TeensyPilot\include\TP_VisionControl\best.pt"
CONF_THRES = 0.25
YOLO_DEVICE = 0 if torch.cuda.is_available() else "cpu"
# ---------------- LOAD MODEL ----------------
model = YOLO(MODEL_PATH)
model.to(YOLO_DEVICE)

def get_YOLO_centroid(frame):

    results = model.predict(
        source=frame,
        task="obb",
        conf=CONF_THRES,
        device=YOLO_DEVICE,
        verbose=False,
        # imgsz=640,
        tracker="bytetrack.yaml"
    )
    # ----- Extract OBB points -----
    if hasattr(results[0], "obb") and results[0].obb is not None:
        # shape: (N, 4, 2)
        # obb_points = results[0].obb.xywhr.cpu().numpy()
        obb_points = results[0].obb.xyxyxyxy.cpu().numpy()
        for i, pts in enumerate(obb_points):
            # pts = [[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
            x = pts[:, 0]
            y = pts[:, 1]

            # -------- CENTROID --------
            cx = int(x.mean())
            cy = int(y.mean())
            # if i==0:
            return cx, cy
    
    return -1, -1
    

"""
# 
# Controller
# 
# Control parameters and Function
#
"""

controller_on = False

c_x = 0.0
c_y = 0.0
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
while True:
    start_time = time.perf_counter()
    # ----- Pygame Event Pump -----
    pygame.event.pump()
    # Read Axes (Sticks: Aileron, Elevator, Throttle, Rudder)
    # Values typically range from -1.0 to 1.0
    axes = [radio.get_axis(i) for i in range(radio.get_numaxes())]
    # Read Buttons (Switches mapped as buttons)
    buttons = [radio.get_button(i) for i in range(radio.get_numbuttons())]
    
    # --------- Get Frame ---------
    if CAMERA_ID:
        ret, frame = cap.read()
        if not ret:
            break
    elif SCREEN_ID:
        sct_img = sct.grab(sct.monitors[SCREEN_ID])
        frame = np.array(sct_img)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    # ----- If AUTO mode is on -----
    roll, pitch, throttle, yaw_rate = 0, 0, 0, 0

    if buttons[2]>0:
        controller_on = True
        # -------- Get target ---------
        if OptFlowMode:
            cx, cy = calcOptFlow(frame)
        elif YOLOMode:
            cx, cy = get_YOLO_centroid(frame)
        elif YOLO_OptFlowMode:
            cx, cy = -1, -1
    
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
    
        roll     = int(axes[0]*RC_STICK_RANGE/2) + RC_STICK_MID
        pitch    = int(axes[1]*RC_STICK_RANGE/2) + RC_STICK_MID
        throttle = int(axes[2]*RC_STICK_RANGE/2) + RC_STICK_MID
        yaw_rate = int(axes[3]*RC_STICK_RANGE/2) + RC_STICK_MID


    # ----- Send CHANNELS_PACKED -----
    if CRSF_initialized:
        ser.write(channelsCrsfToChannelsPacket([roll, pitch, throttle, yaw_rate] + [992 for ch in range(12)]))
    

    ####
    # Display stuff

    # Vision Mode
    # Control Mode
    # Target box/point
    # YOLO stuff

    if controller_on:
        cv2.putText(frame, "AUTO", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "MANUAL", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # joy_center_l = (int(FRAME_WIDTH/2)-55, int(FRAME_HEIGHT/2))
    # joy_center_r = (int(FRAME_WIDTH/2)+55, int(FRAME_HEIGHT/2))

    H_SPACE = 55
    SIZE_HALF = 50
    SIZE = 100
    V_POS = FRAME_HEIGHT*5/6
    H_CENTER = FRAME_WIDTH*1/5
    cv2.rectangle(frame, (int(H_CENTER-H_SPACE-SIZE_HALF), int(V_POS-SIZE_HALF)), (int(FRAME_WIDTH/2-55+SIZE_HALF), int(V_POS+SIZE_HALF)), (200, 200, 200), 2) # center point
    cv2.rectangle(frame, (int(H_CENTER+H_SPACE-SIZE_HALF), int(V_POS-SIZE_HALF)), (int(FRAME_WIDTH/2+55+SIZE_HALF), int(V_POS+SIZE_HALF)), (200, 200, 200), 2) # center point
    
    cv2.circle(frame,    (int(H_CENTER-H_SPACE),                                              int(V_POS)                           ),5, (150, 150, 150), -1) # center point
    cv2.circle(frame,    (int(H_CENTER-H_SPACE+SIZE*(yaw_rate-RC_STICK_MID)/RC_STICK_RANGE),   int(V_POS-SIZE*(throttle-RC_STICK_MID)/RC_STICK_RANGE)),5, (255, 0, 0), -1) # center point
    cv2.circle(frame,    (int(H_CENTER+H_SPACE),                                              int(V_POS)                           ),5, (150, 150, 150), -1) # center point
    cv2.circle(frame,    (int(H_CENTER+H_SPACE+SIZE*(roll-RC_STICK_MID)/RC_STICK_RANGE),       int(V_POS-SIZE*(pitch-RC_STICK_MID)/RC_STICK_RANGE)   ),5, (255, 0, 0), -1) # center point
    

    elapsed = time.perf_counter() - start_time
    cv2.putText(frame, f"Loop Time: {elapsed*1000:0.1f}ms", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
    
    cv2.imshow('Point Tracking', frame)

    k = cv2.waitKey(30) & 0xff # used to be 30
    if k == ord('q'):  # Press 'Esc' to exit
        break
    elif k == ord('c'): # Press 'c' to reset and select a new point
        point_selected = False
        p0 = None


    
    # time.sleep(max(0, LOOP_RATE - elapsed))

cv2.destroyAllWindows()


















# with serial.Serial(PORT, BAUD, timeout=2) as ser, mss.mss() as sct:
#     input = bytearray()
#     # Capture a monitor:
#     monitor = sct.monitors[1]
#     # 1920 1080 half resolution 960 540
#     while True:
        
#         # Get raw pixels from the screen
#         sct_img = sct.grab(monitor)
#         # Convert to a NumPy array for OpenCV
#         img = np.array(sct_img)
#         img = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
#         # Convert BGRA to BGR (OpenCV's default)
#         frame = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

#         if point_selected:
#             frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#             # The first frame after selection is stored as old_gray
#             if old_gray is not None:
#                 p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

#                 # Select good points
#                 if p1 is not None and st.any():
#                     good_new = p1[st == 1]
#                     good_old = p0[st == 1]
                    
#                     # Draw the tracked point
#                     a, b = good_new.ravel()
#                     c, d = good_old.ravel()
#                     target_point = [int(a), int(b)]
#                     cv2.line(frame, (int(a), int(b)), (int(c), int(d)), (255, 0, 0), 2)
#                     cv2.circle(frame, (int(a), int(b)), 5, (0, 255, 0), -1)

#                     # Update the previous points
#                     p0 = good_new.reshape(-1, 1, 2)

#             # Update the previous frame
#             old_gray = frame_gray.copy()
#         else:
#             # While waiting for a click, display the live feed and a message
#             cv2.putText(frame, "Click a point to track", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)


#         cv2.imshow('Point Tracking', frame)


#         k = cv2.waitKey(30) & 0xff
#         if k == ord('q'):  # Press 'Esc' to exit
#             break
#         elif k == ord('c'): # Press 'c' to reset and select a new point
#             point_selected = False
#             p0 = None
#         # sweeping values for testing
#         # roll = int(MIN + (MAX - MIN) * ((time.time() - start_time) % time_period) / time_period)
#         # pitch = int(MIN + (MAX - MIN) * ((time.time() - start_time) % time_period) / time_period)
#         # throttle = int(MIN + (MAX - MIN) * ((time.time() - start_time) % time_period) / time_period)
#         # yaw = int(MIN + (MAX - MIN) * ((time.time() - start_time) % time_period) / time_period)

#         # pitch = 992
#         # throttle = 1200

#         # x, y = target_point if point_selected else (FRAME_WIDTH / 2, FRAME_HEIGHT / 2)

#         # e_x = x - FRAME_WIDTH / 2
#         # e_x /= (FRAME_WIDTH / 2)
#         # e_y = (y - FRAME_HEIGHT / 2)
#         # e_y /= (FRAME_HEIGHT / 2)

#         # roll, pitch, throttle, yaw_rate = controller(e_x, e_y)

#         # # print(f"Mouse cursor is at X: {x}, Y: {y}")
#         # # print(f"Error values are: e_x: {e_x}, e_y: {e_y}")
#         # if ser.in_waiting > 0:
#         #     input.extend(ser.read(ser.in_waiting))
            
#         # else:
#         #     if TX_ENABLED:
#         #         ser.write(channelsCrsfToChannelsPacket([roll, pitch, throttle, yaw_rate] + [992 for ch in range(12)]))
#         #         print("Sent CHANNELS_PACKED")
#         #     time.sleep(0.020)
#         # # print(f"Buffer len: {len(input)}")
#         # while len(input) > 2:
#         #     # This simple parser works with malformed CRSF streams
#         #     # it does not check the first byte for SYNC_BYTE, but
#         #     # instead just looks for anything where the packet length
#         #     # is 4-64 bytes, and the CRC validates
#         #     # print(f"Buffer len: {len(input)}")
#         #     expected_len = input[1] + 2
#         #     if expected_len > 64 or expected_len < 4:
#         #         input = bytearray()
#         #     elif len(input) >= expected_len:
#         #         single = input[:expected_len] # copy out this whole packet
#         #         input = input[expected_len:] # and remove it from the buffer

#         #         if not crsf_validate_frame(single): # single[-1] != crc:
#         #             packet = ' '.join(map(hex, single))
#         #             print(f"crc error: {packet}")
#         #         else:
#         #             handleCrsfPacket(single[2], single)
#         #     else:
#         #         break


