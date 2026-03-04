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

import time, argparse

import math, numpy as np

import serial, mss
import pygame, cv2

import crsf_handler
import controller
import opt_flow

from ultralytics import YOLO
import torch

"""
Shared data
    Frame
    Anotation on?
    Controller on?
"""

# CRSF Serial Setup

CRSF_PORT = 'COM6'
CRSF_BAUD = 921600
# ELRS TX

# TX_ENABLED = True
crsf = None
CRSF_initialized = False
try:
    crsf = crsf_handler.CRSFHandler(port=CRSF_PORT, baudrate=CRSF_BAUD)
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

cx = 0
cy = 0


"""
#
# Create a window and bind the mouse callback function to it
#
"""
cv2.namedWindow('Point Tracking', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Point Tracking', int(FRAME_WIDTH), int(FRAME_HEIGHT))
cv2.setMouseCallback('Point Tracking', opt_flow.select_point)


"""
# 
# YOLO ID and Tracking
# 
"""

MODEL_PATH  = "yolo_liftoff_weights.pt"
CONF_THRES  = 0.25
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
    # TODO pick a target (currently it returns centroid of the first target)
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
controller_on = False
Vision_Switch = 1
roll, pitch, throttle, yaw_rate = 0, 0, 0, 0
channels = []

while True:
    start_time = time.perf_counter()
    # ----- Pygame Event Pump -----
    pygame.event.pump()
    if radio_initialized:
        # Read Axes (Sticks: Aileron, Elevator, Throttle, Rudder)
        # Values typically range from -1.0 to 1.0
        # Read Buttons (Switches mapped as buttons)
        # 8 channels:   r,              p,          t,          y,      arm-SF,     flightmode-SC,     Vision SD,   VTX-powerlevel SA,
        # buttons:      VTXon/off SB,   VTX-Ch-6P,  Control-SH
        axes = [radio.get_axis(i) for i in range(radio.get_numaxes())]
        buttons = [radio.get_button(i) for i in range(radio.get_numbuttons())]

        roll     = int(axes[0]*controller.RC_STICK_RANGE/2) + controller.RC_STICK_MID
        pitch    = int(axes[1]*controller.RC_STICK_RANGE/2) + controller.RC_STICK_MID
        throttle = int(axes[2]*controller.RC_STICK_RANGE/2) + controller.RC_STICK_MID
        yaw_rate = int(axes[3]*controller.RC_STICK_RANGE/2) + controller.RC_STICK_MID

        arm_3sw       = int(axes[4]*controller.RC_STICK_RANGE/2) + controller.RC_STICK_MID
        fmode_3sw     = int(axes[5]*controller.RC_STICK_RANGE/2) + controller.RC_STICK_MID
        vision_3sw    = int(axes[6]*controller.RC_STICK_RANGE/2) + controller.RC_STICK_MID
        vtx_power_3sw = int(axes[7]*controller.RC_STICK_RANGE/2) + controller.RC_STICK_MID

        VTX_2sw     = int(buttons[0]*controller.RC_STICK_RANGE) + controller.RC_STICK_MIN
        VTXCh_2sw   = int(buttons[1]*controller.RC_STICK_RANGE) + controller.RC_STICK_MIN
        control_2sw = int(buttons[2]*controller.RC_STICK_RANGE) + controller.RC_STICK_MIN

        # Vision_Switch = int(buttons[0]*controller.RC_STICK_RANGE/2) + controller.RC_STICK_MID
        AUTO_Switch = True if buttons[2]>0 else False
        Vision_Switch = round(axes[6]+1) # modes: 0 1 2

        channels = [roll, pitch, throttle, yaw_rate, arm_3sw, fmode_3sw, vision_3sw, vtx_power_3sw] + [VTX_2sw, VTXCh_2sw, control_2sw] + [992 for ch in range(5)]
        # for i in range(8):
        #     channels.append(int(axes[i]*controller.RC_STICK_RANGE/2) + controller.RC_STICK_MID)
        # for i in range(8):
        #     channels.append(int(buttons[i]*controller.RC_STICK_RANGE/2) + controller.RC_STICK_MID)


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

    # cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT), cv2.INTER_AREA)

    # print(Vision_Switch)
    # ----- If AUTO mode is on -----
    if Vision_Switch>0:
        # -------- Get target ---------
        if Vision_Switch == 1:
            cx, cy = opt_flow.calcOptFlow(frame)
        elif Vision_Switch == 2:
            cx, cy = get_YOLO_centroid(frame)
        elif YOLO_OptFlowMode:
            cx, cy = -1, -1

        if AUTO_Switch:
            
            if cx != -1 and cy != -1:
                e_x = cx - FRAME_WIDTH / 2
                e_x /= (FRAME_WIDTH / 2)
                e_y = cy - FRAME_HEIGHT / 2
                e_y /= (FRAME_HEIGHT / 2)
                e_x = 0
                e_y = 0
                roll, pitch, throttle, yaw_rate = controller.controller(e_x, e_y)
                channels = [roll, pitch, throttle, yaw_rate] + [992 for ch in range(12)]
        else:
            controller_on = False
            # with vis_con_data["lock"]:
            #     vis_con_data["controller_on"] = False


    # ----- Send CHANNELS_PACKED -----
    if CRSF_initialized:
        crsf.write(channels)
        # crsf.write_stabilize(roll, pitch, throttle, yaw_rate)
        # ser.write(channelsCrsfToChannelsPacket([roll, pitch, throttle, yaw_rate] + [992 for ch in range(12)]))
    

    ####
    # Display stuff

    # Vision Mode
    # Control Mode
    # Target box/point
    # YOLO stuff

    fontFace = cv2.FONT_HERSHEY_SIMPLEX
    textHeight = int(FRAME_HEIGHT/30)
    thickness = 2
    fontScale = cv2.getFontScaleFromHeight(fontFace, textHeight, thickness)

    textHeight2 = int(FRAME_HEIGHT/100)
    thickness2 = 1
    fontScale2 = cv2.getFontScaleFromHeight(fontFace, textHeight, thickness2)

    TextBox_x = int(FRAME_HEIGHT/20)
    TextBox_y = int(FRAME_HEIGHT/20)

    TX_Ch_Textspace_x = int((FRAME_WIDTH-2*TextBox_x)/5)
    TX_Ch_Textbox_y = int(TextBox_y + 3.5*textHeight)

    if AUTO_Switch:
        color = (100, 100, 100)
        if Vision_Switch>0:
            color = (0, 255, 0)
        cv2.putText(frame, "AUTO", (TextBox_x, TextBox_y), fontFace, fontScale, color, thickness)
    else:
        cv2.putText(frame, "MANUAL", (TextBox_x, TextBox_y), fontFace, fontScale, (0, 0, 255), thickness)

    if Vision_Switch==1:
        cv2.putText(frame, "Optical Flow", (TextBox_x, int(TextBox_y + 1.5*textHeight)), fontFace, fontScale, (255, 100, 0), thickness)
    elif Vision_Switch==2:
        cv2.putText(frame, "YOLO", (TextBox_x, int(TextBox_y + 1.5*textHeight)), fontFace, fontScale, (0, 255, 0), thickness)
    else:
        cv2.putText(frame, "Vision OFF", (TextBox_x, int(TextBox_y + 1.5*textHeight)), fontFace, fontScale, (0, 0, 255), thickness)

    # arm-SF,     flightmode-SC,  --Vision SD--,   VTX-powerlevel SA,
    # buttons:      VTXon/off SB,   VTX-Ch-6P,  Control-SH
    if arm_3sw>controller.RC_STICK_MID:
        cv2.putText(frame, "Arm", (TextBox_x, TX_Ch_Textbox_y), fontFace, fontScale2, (0, 255, 0), thickness2)
    else:
        cv2.putText(frame, "Disarm", (TextBox_x, TX_Ch_Textbox_y), fontFace, fontScale2, (0, 0, 255), thickness2)
    
    fmode_text = None
    if fmode_3sw<controller.RC_STICK_MID-controller.RC_STICK_RANGE/4:
        fmode_text = "| FlM 1"
    elif fmode_3sw<controller.RC_STICK_MID+controller.RC_STICK_RANGE/4:
        fmode_text = "| FlM 2"
    else:
        fmode_text = "| FlM 3"
    cv2.putText(frame, fmode_text, (TextBox_x+TX_Ch_Textspace_x, TX_Ch_Textbox_y), fontFace, fontScale2, (0, 255, 255), thickness2)
    
    VTX_power_text = None
    if vtx_power_3sw<controller.RC_STICK_MID-controller.RC_STICK_RANGE/4:
        VTX_power_text = "| VTx p1"
    elif vtx_power_3sw<controller.RC_STICK_MID+controller.RC_STICK_RANGE/4:
        VTX_power_text = "| VTx p2"
    else:
        VTX_power_text = "| Vtx p3"
    cv2.putText(frame, VTX_power_text, (TextBox_x+2*TX_Ch_Textspace_x, TX_Ch_Textbox_y), fontFace, fontScale2, (255, 0, 255), thickness2)
    
    if VTX_2sw>controller.RC_STICK_MID:
        cv2.putText(frame, "| VTX On", (TextBox_x+3*TX_Ch_Textspace_x, TX_Ch_Textbox_y), fontFace, fontScale2, (0, 255, 0), thickness2)
    else:
        cv2.putText(frame, "| VTX Off", (TextBox_x+3*TX_Ch_Textspace_x, TX_Ch_Textbox_y), fontFace, fontScale2, (0, 0, 255), thickness2)
    
    if VTXCh_2sw>controller.RC_STICK_MID:
        cv2.putText(frame, "| VTX Ch1", (TextBox_x+4*TX_Ch_Textspace_x, TX_Ch_Textbox_y), fontFace, fontScale2, (0, 255, 255), thickness2)
    else:
        cv2.putText(frame, "| VTX Ch6", (TextBox_x+4*TX_Ch_Textspace_x, TX_Ch_Textbox_y), fontFace, fontScale2, (0, 255, 255), thickness2)
    



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
    cv2.circle(frame, (int(H_CENTER-H_SPACE+SIZE*(yaw_rate-controller.RC_STICK_MID)/controller.RC_STICK_RANGE),  int(V_CENTER-SIZE*(throttle-controller.RC_STICK_MID)/controller.RC_STICK_RANGE)),5, (255, 0, 0), -1) # center point
    cv2.circle(frame, (int(H_CENTER+H_SPACE),                                              int(V_CENTER)                           ),5, (150, 150, 150), -1) # center point
    cv2.circle(frame, (int(H_CENTER+H_SPACE+SIZE*(roll-controller.RC_STICK_MID)/controller.RC_STICK_RANGE),      int(V_CENTER-SIZE*(pitch-controller.RC_STICK_MID)/controller.RC_STICK_RANGE)   ),5, (255, 0, 0), -1) # center point
    
    if opt_flow.point_selected:
        cv2.circle(frame, (int(opt_flow.selected_point[0]), int(opt_flow.selected_point[1])), 5, (150, 150, 150), -1) # center point
        cv2.circle(frame, (int(cx), int(cy)), 5, (255, 0, 0), -1) # center point

    
    
    elapsed = time.perf_counter() - start_time
    cv2.putText(frame, f"| dt: {elapsed*1000:2.0f}ms", (TextBox_x+TX_Ch_Textspace_x, TextBox_y), fontFace, fontScale, (255, 255, 0), thickness)
    
    cv2.imshow('Point Tracking', frame)

    k = cv2.waitKey(30) & 0xff # used to be 30
    if k == ord('q'):  # Press 'Esc' to exit
        break
    elif k == ord('c'): # Press 'c' to reset and select a new point
        point_selected = False
        p0 = None


    
    # time.sleep(max(0, LOOP_RATE - elapsed))

cv2.destroyAllWindows()

