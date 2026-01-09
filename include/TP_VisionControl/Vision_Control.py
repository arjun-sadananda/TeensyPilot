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

cx = 0
cy = 0

# Ideally Use LOS instead of pixel errors directly
e_x_prev = 0.0
e_y_prev = 0.0
e_y_integral = 0.0

roll, pitch, throttle, yaw_rate = 0, 0, 0, 0


def controller(e_x, e_y):
    global e_x_prev, e_y_prev, prev_time, e_y_integral
    
    dt = shared_data["vision_control_dt"]
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
JOY_DEVICE_ID = 1  # EdgeTX radio joystick ID

# ELRS TX
CRSF_PORT = 'COM19'
CRSF_BAUD = 921600

# CAMERA / SCREEN
SCREEN_ID = 1       # Set to None to use camera instead of screen capture
CAMERA_ID = None    # Set to None to use screen capture instead of camera

# YOLO OBB
MODEL_PATH = r"C:\Users\Arjun\Documents\GitHub\TeensyPilot\include\TP_VisionControl\best.pt"
CONF_THRES = 0.25
YOLO_DEVICE = 0 if torch.cuda.is_available() else "cpu"



# ----------- YOLO WORKER PROCESS --------------
def yolo_worker(input_q, output_q):
    model = YOLO(MODEL_PATH)
    model.to(YOLO_DEVICE)
    print("🎥 Real-time OBB inference started. Press 'Q' to quit.")

    while True:
        frame = input_q.get() # Wait for a frame to process
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
            # obb_points = results[0].obb.xyxyxyxy.cpu().numpy()
            obb_points = results[0].obb.xywhr.cpu().numpy()

            # for pts in obb_points:
            box = obb_points[0]
            # pts = [[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
            # x = pts[:, 0]
            # y = pts[:, 1]
            x, y, w, h, r = box[0] - box[2]/2, box[1] - box[3]/2, box[2], box[3], box[4]

            output_q.put((int(x), int(y), int(w), int(h), int(r)))
        # If OBB found, send the first detection back to the main thread
        # if hasattr(results[0], "obb") and len(results[0].obb) > 0:
        #     # Extract bounding box for OpenCV tracker (x, y, w, h)
        #     # We convert OBB to standard XYWH for the lightweight tracker
        #     box = results[0].obb.xywh[0].cpu().numpy()
        #     x, y, w, h = box[0] - box[2]/2, box[1] - box[3]/2, box[2], box[3]
        #     output_q.put((int(x), int(y), int(w), int(h)))

# ---------------- MAIN LOOP ----------------

shared_data = {
    "controller_on": False,
    "tracking_initialized": False,
    "serial_control_dt": 0.0,
    "vision_control_dt": 0.0,
    "cx": 0,
    "cy": 0,
    "roll": 0,
    "pitch": 0,
    "throttle": 0,
    "yaw_rate": 0,
    "lock": threading.Lock()
    
}

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
        with shared_data["lock"]:
            shared_data["serial_control_dt"] = time.time() - sc_prev_time
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
            with shared_data["lock"]:
                shared_data["controller_on"] = True
            
            e_x = cx - FRAME_WIDTH / 2
            e_x /= (FRAME_WIDTH / 2)
            e_y = cy - FRAME_HEIGHT / 2
            e_y /= (FRAME_HEIGHT / 2)
            roll, pitch, throttle, yaw_rate = controller(e_x, e_y)
        else:
            with shared_data["lock"]:
                shared_data["controller_on"] = False
            
            roll = int(axes[0]*RC_STICK_RANGE/2) + RC_STICK_MID
            pitch = int(axes[1]*RC_STICK_RANGE/2) + RC_STICK_MID
            throttle = int(axes[2]*RC_STICK_RANGE/2) + RC_STICK_MID
            yaw_rate = int(axes[3]*RC_STICK_RANGE/2) + RC_STICK_MID

        # ----- Send CHANNELS_PACKED -----
        crsf.write(roll, pitch, throttle, yaw_rate)

        # elapsed = time.perf_counter() - start_time
        # time.sleep(max(0, LOOP_RATE - elapsed))
    
    # Cleanup?

def vision_control_loop():

    # CAMERA / SCREEN
    if CAMERA_ID is not None:
        cap = cv2.VideoCapture(CAMERA_ID)
        # if not cap.isOpened():
        #     raise RuntimeError("Cannot open camera")
    elif SCREEN_ID is not None:
        sct = mss.mss()

    # IPC Queues
    frame_to_yolo = Queue(maxsize=1)
    results_from_yolo = Queue(maxsize=1)

    # Start YOLO Process
    p = Process(target=yolo_worker, args=(frame_to_yolo, results_from_yolo), daemon=True)
    p.start()

    # Shared Vision/Control Data
    target_box = None
    tracker = cv2.Tracker()

    vc_prev_time = time.time()
    last_yolo_time = 0
    yolo_dt = 0  # seconds between YOLO inferences
    while True:
        with shared_data["lock"]:
            shared_data["vision_control_dt"] = time.time() - vc_prev_time
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
        annotated_frame = frame.copy()
       
        # 1. Send frame to YOLO (non-blocking)
        if frame_to_yolo.empty():
            frame_to_yolo.put(frame)

        # 2. Check for YOLO updates (Correction)
        if not results_from_yolo.empty():
            new_box = results_from_yolo.get()
            new_box = (new_box[0], new_box[1], new_box[2], new_box[3])  # x, y, w, h
            tracker = cv2.Tracker() # Reset tracker
            tracker.init(frame, new_box)
            with shared_data["lock"]:
                shared_data["tracking_initialized"] = True
            
            yolo_dt = time.time() - last_yolo_time
            last_yolo_time = time.time()
            
        
        # 3. Fast Tracking
        if shared_data["tracking_initialized"]:
            success, bbox = tracker.update(frame)
            if success:
                # DRAW & UPDATE CONTROLLER DATA
                x, y, w, h = [int(v) for v in bbox]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # Update controller errors here...

                # -------- CENTROID --------
                cx = int(x.mean())
                cy = int(y.mean())
                # draw OBB (optional)
                cv2.polylines(annotated_frame, [pts.astype(int)], isClosed=True, color=(0, 255, 0), thickness=2)

                # draw centroid
                cv2.circle(annotated_frame, (cx, cy), radius=3, color=(0, 0, 255), thickness=-1)

                with shared_data["lock"]:
                    shared_data["cx"] = cx
                    shared_data["cy"] = cy
        # # 1. TRACKING & VISION LOGIC
        # if tracking:
        #     success, bbox = tracker.update(frame)
        #     if success:
        #         # Calculate errors and run Controller math here
        #         # r, p, t, y = controller(e_x, e_y)
                
        #         with shared_data["lock"]:
        #             shared_data["roll"], shared_data["pitch"] = r, p
        #             # Update other shared channels...

        # # 2. DISPLAY (Vision Thread handles UI)
        # cv2.imshow("Drone View", frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'): break
        
        if shared_data["controller_on"]:
            cv2.putText(annotated_frame, "AUTO", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) 
        else:
            cv2.putText(annotated_frame, "MANUAL", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) 

        # TODO Display more info - YOLO DT, Tracker DT, Control DT
        cv2.putText(annotated_frame, str(round(shared_data["serial_control_dt"]*10**3, 2))+" ms", (200, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) 
        cv2.putText(annotated_frame, str(round(shared_data["vision_control_dt"]*10**3, 2))+" ms", (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) 
        cv2.putText(annotated_frame, str(round(yolo_dt*10**3, 2))+" ms", (600, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2) 
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

# ---------------- START THREADS ----------------
stop_event = threading.Event()
vision_thread = threading.Thread(target=vision_control_loop, daemon=True)
vision_thread.start()

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
