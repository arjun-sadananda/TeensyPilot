"""
# 
# Optical Flow Point Tracking 
# 
"""

import cv2
import numpy as np

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