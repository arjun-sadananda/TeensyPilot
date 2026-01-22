from multiprocessing import Process, Queue
import sys
import threading

import cv2
import mss
import numpy as np
import pygame
from ultralytics import YOLO
import torch

import serial
import time
import argparse
from enum import IntEnum

import math
import crsf_handler


##
"""
Modes:  Manual Pilot (SG Low (3), SH Low (2))
            Fast Forward Joy Stream to CRSF
            Run YOLO OBB and Visualise only
        Target Lock (SH Low to High Edge(2))
            Run Tracker
        Auto Pilot
Threads:  
        Radio Input and CRSF Output (___Hz)
        Image Capture, Tracker (If ON), Controller (If ON) and Annotated Display
            (___Hz/___Hz/___Hz)
        YOLO OBB (___Hz)
"""

# ---------------- (Vision Based) Controller ----------------

RC_STICK_MIN = 172
RC_STICK_MAX = 1811
RC_STICK_RANGE = RC_STICK_MAX - RC_STICK_MIN        # 1811-992 = 819
RC_STICK_MID = (RC_STICK_MIN + RC_STICK_MAX) // 2   # (1811+172)/2 = 992

ANGLE_LIMIT = 55.0      # degrees
CAMERA_ANGLE = 25.0     # degrees

# 848, 678 640*2
FRAME_WIDTH  = 1920*3//4
FRAME_HEIGHT = 1080*3//4

HOVER_THROTTLE_PC = 0.27  # Portion of throttle range to hover at level flight

KP_ROLL     = RC_STICK_RANGE*.6
KP_YAW_RATE = RC_STICK_RANGE*.6
KP_PITCH    = RC_STICK_RANGE*.5
KP_THROTTLE = RC_STICK_RANGE*1
KD_ROLL     = 0.0
KD_YAW_RATE = 0.0
KD_PITCH    = 0.0
KD_THROTTLE = 0.0
KI_THROTTLE = RC_STICK_RANGE*1

PITCH_BIAS = RC_STICK_RANGE/2 * (CAMERA_ANGLE / ANGLE_LIMIT)

def controller(e_x, e_y):
    global e_x_prev, e_y_prev, prev_time, e_y_integral
    
    dt = vis_con_data["vision_control_dt"]
    if dt >0:
        d_e_x = (e_x - e_x_prev)/dt
        d_e_y = (e_y - e_y_prev)/dt
    else:
        d_e_x = 0.0
        d_e_y = 0.0

    e_x_prev = e_x
    e_y_prev = e_y

    if abs(e_y) < .25:
        e_y_integral += e_y * dt  # Simple integral term, could be accumulated over time
    else:
        e_y_integral = 0.0  # Reset integral if error is large

    # convert e_y to angle (use LOS)
    
    # Total control outputs
    u_roll     = RC_STICK_MID + KP_ROLL * e_x     + KD_ROLL * d_e_x
    u_yaw_rate = RC_STICK_MID + KP_YAW_RATE * e_x + KD_YAW_RATE * d_e_x
    u_pitch    = RC_STICK_MID + KP_PITCH * e_y    + KD_PITCH * d_e_y    + PITCH_BIAS

    # replace pitch with tilt angle for throttle feedforward
    pitch = ANGLE_LIMIT * (u_pitch - RC_STICK_MID) / (RC_STICK_RANGE/2)
    throttle_ff = RC_STICK_MIN + (RC_STICK_RANGE * HOVER_THROTTLE_PC / math.cos(math.radians(pitch)))  # Adjust as needed
    u_throttle  = throttle_ff - KP_THROTTLE * e_y   + KD_THROTTLE * d_e_y    - KI_THROTTLE * e_y_integral
    
    # Clip to valid range
    u_roll      = max(RC_STICK_MIN, min(RC_STICK_MAX, int(u_roll)))
    u_yaw_rate  = max(RC_STICK_MIN, min(RC_STICK_MAX, int(u_yaw_rate)))
    u_pitch     = max(RC_STICK_MIN, min(RC_STICK_MAX, int(u_pitch)))
    u_throttle  = max(RC_STICK_MIN, min(RC_STICK_MAX, int(u_throttle)))

    # # Clip to slow range
    # RC_STICK_CAP_MIN = RC_STICK_MID - RC_STICK_RANGE/4
    # RC_STICK_CAP_MAX = RC_STICK_MID + RC_STICK_RANGE/4
    # u_roll      = max(RC_STICK_CAP_MIN, min(RC_STICK_CAP_MAX, int(u_roll)))
    # u_yaw_rate  = max(RC_STICK_CAP_MIN, min(RC_STICK_CAP_MAX, int(u_yaw_rate)))
    # u_pitch     = max(RC_STICK_CAP_MIN, min(RC_STICK_CAP_MAX, int(u_pitch)))
    # u_throttle  = max(RC_STICK_MIN, min(RC_STICK_MAX, int(u_throttle)))

    return u_roll, u_pitch, u_throttle, u_yaw_rate


# ---------------- CONFIG ----------------

# JOY RX
JOY_DEVICE_ID = 0  # EdgeTX radio joystick ID

# ELRS TX
CRSF_PORT = 'COM19'
CRSF_BAUD = 921600

# CAMERA / SCREEN
SCREEN_ID = 2       # Set to None to use camera instead of screen capture
CAMERA_ID = None    # Set to None to use screen capture instead of camera

# YOLO OBB
MODEL_PATH = r"C:\Users\Arjun\Documents\GitHub\TeensyPilot\include\TP_VisionControl\best.pt"
CONF_THRES = 0.25
YOLO_DEVICE = 0 if torch.cuda.is_available() else "cpu"


# ---------------- MAIN LOOP ----------------
# Ideally Use LOS instead of pixel errors directly

vis_con_data = {
    "controller_on": False,
    # "tracking_initialized": False,
    "serial_control_dt": 0.0,
    "vision_control_dt": 0.0,
    "cx": 0,
    "cy": 0,
    "e_x_prev": 0.0,
    "e_y_prev": 0.0,
    "e_y_integral": 0.0,
    # "roll": 0,
    # "pitch": 0,
    # "throttle": 0,
    # "yaw_rate": 0,
    "lock": threading.Lock()
}

yolo_data = {
    "frame": None,
    "obb_boxes": None,
    "done": True,
    "lock": threading.Lock()
}
# ----------- YOLO WORKER PROCESS --------------
def yolo_loop():
    model = YOLO(MODEL_PATH)
    model.to(YOLO_DEVICE)
    print("🎥 Real-time OBB inference started. Press 'Q' to quit.")
    
    # TODO: Exception handling
    while True:
        if yolo_data["done"]:
            time.sleep(0.01)
            continue

        with yolo_data["lock"]:
            frame = yolo_data["frame"]
        
        if frame is None:
            time.sleep(0.01)
            continue

        # ----- YOLO OBB Inference -----
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
            with yolo_data["lock"]:
                yolo_data["obb_boxes"] = obb_points
                yolo_data["done"] = True
              

            # obb_points = results[0].obb.xywhr.cpu().numpy()
            # box = obb_points[0]

            # x, y, w, h, r = box[0] - box[2]/2, box[1] - box[3]/2, box[2], box[3], box[4]


def vision_feedback_loop():

    # CAMERA / SCREEN
    if CAMERA_ID is not None:
        cap = cv2.VideoCapture(CAMERA_ID)
        # if not cap.isOpened():
        #     raise RuntimeError("Cannot open camera")
    elif SCREEN_ID is not None:
        sct = mss.mss()

    # Shared Vision/Control Data
    tracking_initialized = False
    target_box = None
    # tracker = cv2.Tracker()
    tracker = cv2.legacy.TrackerKCF_create()

    vc_prev_time = time.time()
    last_yolo_time = 0
    yolo_dt = 0  # seconds between YOLO inferences

    frame = None
    annotated_frame = None
    
    # Opt Flow Tracking
    p0 = None

    # Parameters for Lucas-Kanade optical flow
    lk_params = dict(winSize=(50, 50),
                    maxLevel=2,
                    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    old_gray = None
    
    delta = np.array([[[0, 0]]], dtype=np.float32)

    while True:
        with vis_con_data["lock"]:
            vis_con_data["vision_control_dt"] = time.time() - vc_prev_time
        vc_prev_time = time.time()

        if CAMERA_ID:
            ret, frame = cap.read()
            if not ret:
                break
        elif SCREEN_ID:
            sct_img = sct.grab(sct.monitors[SCREEN_ID])
            frame = np.array(sct_img)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        
        new_dimensions = (FRAME_WIDTH, FRAME_HEIGHT)
        frame = cv2.resize(frame, new_dimensions, interpolation=cv2.INTER_AREA)
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        annotated_frame = frame.copy()

        
        # 1. Send frame to YOLO (non-blocking)
        if yolo_data["done"]:
            with yolo_data["lock"]:
                yolo_data["frame"] = frame.copy()
                yolo_data["done"] = False
                # 2. Get YOLO updates (Correction)
                new_boxes = yolo_data["obb_boxes"]
                # new_box = yolo_data["obb_box"]
            
            if new_boxes is not None:
                for i, pts in enumerate(new_boxes):
                    # pts = [[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
                    x = pts[:, 0]
                    y = pts[:, 1]

                    # -------- CENTROID --------
                    cx = int(x.mean())
                    cy = int(y.mean())

                    if i==0:
                        with vis_con_data["lock"]:
                            vis_con_data["cx"] = cx
                            vis_con_data["cy"] = cy
                        target_box = (min(x), min(y), max(x)-min(x), max(y)-min(y))
                        # if not tracking_initialized:
                            # tracker.init(frame, target_box)
                        old_gray = frame_gray.copy()
                        p0 = np.array([[[cx, cy]]], dtype=np.float32)
                        p0 += delta
                        delta = np.array([[[0, 0]]], dtype=np.float32)

                        tracking_initialized = True
                        cv2.circle(annotated_frame, (cx, cy), radius=10, color=(255, 0, 255), thickness=1)

                    # draw OBB (optional)
                    cv2.polylines(annotated_frame, [pts.astype(int)], isClosed=True, color=(0, 255, 0), thickness=2)

                    # draw centroid
                    cv2.circle(annotated_frame, (cx, cy), radius=3, color=(0, 0, 255), thickness=-1)
                yolo_dt = time.time() - last_yolo_time
                last_yolo_time = time.time()
            # if new_box is not None:
            #     new_box = (new_box[0], new_box[1], new_box[2], new_box[3])  # x, y, w, h
            #     # tracker = cv2.Tracker() # Reset tracker
            #     # tracker.init(frame, new_box)
            #     with shared_data["lock"]:
            #         shared_data["tracking_initialized"] = True
            # 
            #     x, y, w, h = [int(v) for v in new_box]
            # 
            #     yolo_dt = time.time() - last_yolo_time
            #     last_yolo_time = time.time()
            #     cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
        
        # 3. Fast Tracking
        if tracking_initialized:
            # ok, bbox = tracker.update(frame)
            # if ok:
            #     cv2.rectangle(annotated_frame, (int(bbox[0]), int(bbox[1])), (int(bbox[0]+bbox[2]), int(bbox[1]+bbox[3])), (255, 0, 0), 2)
            #     # Update controller errors here...
            #     cx = int(bbox[0] + bbox[2]/2)
            #     cy = int(bbox[1] + bbox[3]/2)
            #     # draw centroid
            #     cv2.circle(annotated_frame, (cx, cy), radius=3, color=(0, 0, 255), thickness=-1)
            #     with vis_con_data["lock"]:
            #         vis_con_data["cx"] = cx
            #         vis_con_data["cy"] = cy

            p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

            # Select good points
            if p1 is not None and st.any():

                good_new = p1[st == 1]
                good_old = p0[st == 1]
                
                # Draw the tracked point
                cx,     cy     = good_new.ravel()
                cx_old, cy_old = good_old.ravel()
                
                delta += (good_new - good_old)

                target_point = [int(cx), int(cy)]
                cv2.line(annotated_frame, (int(cx), int(cy)), (int(cx_old), int(cy_old)), (255, 0, 0), 2)
                cv2.circle(annotated_frame, (int(cx), int(cy)), 5, (0, 255, 0), -1)
                # Update the previous points
                p0 = good_new.reshape(-1, 1, 2)

            old_gray = frame_gray.copy()

            # success, bbox = tracker.update(frame)
            # if success:
            #     # DRAW & UPDATE CONTROLLER DATA
            #     x, y, w, h = [int(v) for v in bbox]
            #     cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            #     # Update controller errors here...

            #     # -------- CENTROID --------
            #     cx = int(x.mean())
            #     cy = int(y.mean())
            #     # draw OBB (optional)
            #     cv2.polylines(annotated_frame, [pts.astype(int)], isClosed=True, color=(0, 255, 0), thickness=2)

            #     # draw centroid
            #     cv2.circle(annotated_frame, (cx, cy), radius=3, color=(0, 0, 255), thickness=-1)

            #     with shared_data["lock"]:
            #         shared_data["cx"] = cx
            #         shared_data["cy"] = cy

        if vis_con_data["controller_on"]:
            cv2.putText(annotated_frame, "AUTO", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) 
        else:
            cv2.putText(annotated_frame, "MANUAL", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) 

        # TODO Display more info - YOLO DT, Tracker DT, Control DT
        cv2.putText(annotated_frame, str(round(vis_con_data["serial_control_dt"]*10**3, 2))+" ms", (200, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) 
        cv2.putText(annotated_frame, str(int(1/vis_con_data["serial_control_dt"]))+" Hz", (200, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) 
        cv2.putText(annotated_frame, str(round(vis_con_data["vision_control_dt"]*10**3, 2))+" ms", (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) 
        cv2.putText(annotated_frame, str(int(1/vis_con_data["vision_control_dt"]))+" Hz", (400, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) 
        if yolo_dt>0:
            cv2.putText(annotated_frame, str(round(yolo_dt*10**3, 2))+" ms", (600, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) 
            cv2.putText(annotated_frame, str(int(1/yolo_dt))+" Hz", (600, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) 
        cv2.imshow("YOLO OBB with Centroid", annotated_frame)
        # time.sleep(0.020)

        # ----- Exit and Cleanup -----
        if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q')):
            break

    # ---------------- CLEANUP ----------------
    if CAMERA_ID:
        cap.release() 
    cv2.destroyAllWindows()
    print("Camera inference stopped")


def serial_control_loop():
    # JOY RX
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No EdgeTX radio found. Ensure it is in USB Joystick mode.")
        exit()
    radio = pygame.joystick.Joystick(JOY_DEVICE_ID)
    radio.init()
    print(f"Connected to: {radio.get_name()}")

    # ELRS TX
    crsf = crsf_handler.CRSFHandler(port=CRSF_PORT, baudrate=CRSF_BAUD)
    
    # LOOP_RATE = 0.01 # 100Hz
    sc_prev_time = time.time()
    while True:
        with vis_con_data["lock"]:
            vis_con_data["serial_control_dt"] = time.time() - sc_prev_time
        sc_prev_time = time.time()

        # ----- Pygame Event Pump -----
        pygame.event.pump()
        # Read Axes (Sticks: Aileron, Elevator, Throttle, Rudder)
        # Values typically range from -1.0 to 1.0
        axes = [radio.get_axis(i) for i in range(radio.get_numaxes())]
        # Read Buttons (Switches mapped as buttons)
        buttons = [radio.get_button(i) for i in range(radio.get_numbuttons())]
        
        # TODO Also check if tracker has initialized
        if buttons[2]>0:
            with vis_con_data["lock"]:
                vis_con_data["controller_on"] = True
                cx = vis_con_data["cx"]
                cy = vis_con_data["cy"]
            
            e_x = cx - FRAME_WIDTH / 2
            e_x /= (FRAME_WIDTH / 2)
            e_y = cy - FRAME_HEIGHT / 2
            e_y /= (FRAME_HEIGHT / 2)
            roll, pitch, throttle, yaw_rate = controller(e_x, e_y)
        else:
            with vis_con_data["lock"]:
                vis_con_data["controller_on"] = False
            
            roll = int(axes[0]*RC_STICK_RANGE/2) + RC_STICK_MID
            pitch = int(axes[1]*RC_STICK_RANGE/2) + RC_STICK_MID
            throttle = int(axes[2]*RC_STICK_RANGE/2) + RC_STICK_MID
            yaw_rate = int(axes[3]*RC_STICK_RANGE/2) + RC_STICK_MID

        # ----- Send CHANNELS_PACKED -----
        crsf.write(roll, pitch, throttle, yaw_rate)

        # elapsed = time.perf_counter() - start_time
        # time.sleep(max(0, LOOP_RATE - elapsed))
    
    # Cleanup?



# ---------------- START THREADS ----------------
stop_event = threading.Event()
vision_thread = threading.Thread(target=vision_feedback_loop, daemon=True)
vision_thread.start()

yolo_thread = threading.Thread(target=yolo_loop, daemon=True)
yolo_thread.start()


# try:
serial_control_loop()
# except KeyboardInterrupt:
#     # This block is executed when Ctrl+C is pressed
#     print("\nKeyboard Interrupt received.")
#     print("Performing cleanup actions...")
#     # Add your specific cleanup code here (e.g., closing files, releasing resources)
time.sleep(5)
stop_event.set() # Set the event to signal termination
vision_thread.join() # Wait for the thread to finish
print("Cleanup complete. Exiting gracefully.")
sys.exit(0) # Exit the program with a status code of 0 (success)
