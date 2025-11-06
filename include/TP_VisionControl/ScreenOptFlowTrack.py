import cv2
import numpy as np
import mss



# Global variables to store the selected points
p0 = None
# Flag to indicate if a point has been selected
point_selected = False

old_gray = None

def select_point(event, x, y, flags, param):
    global p0, point_selected
    if event == cv2.EVENT_LBUTTONDOWN:
        # Store the clicked point as a NumPy array of float32,
        # formatted correctly for calcOpticalFlowPyrLK
        p0 = np.array([[[x, y]]], dtype=np.float32)
        point_selected = True
        print(f"Tracking point selected at: ({x}, {y})")


# Parameters for Lucas-Kanade optical flow
lk_params = dict(winSize=(15, 15),
                maxLevel=2,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))


# Create a window and bind the mouse callback function to it
cv2.namedWindow('Point Tracking')
cv2.setMouseCallback('Point Tracking', select_point)

with mss.mss() as sct:
    # Define the region of the screen to capture (optional)
    # monitor = {"top": 100, "left": 100, "width": 800, "height": 600} 
    # Or capture the primary monitor: 
    monitor = sct.monitors[2]
    # 1920 1080 half resolution 960 540

    while True:
        # Get raw pixels from the screen
        sct_img = sct.grab(monitor)

        # Convert to a NumPy array for OpenCV
        img = np.array(sct_img)
        img = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        # Convert BGRA to BGR (OpenCV's default)
        frame = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

        if point_selected:
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # The first frame after selection is stored as old_gray
            if old_gray is not None:
                p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

                # Select good points
                if p1 is not None and st.any():
                    good_new = p1[st == 1]
                    good_old = p0[st == 1]
                    
                    # Draw the tracked point
                    a, b = good_new.ravel()
                    c, d = good_old.ravel()
                    cv2.line(frame, (int(a), int(b)), (int(c), int(d)), (255, 0, 0), 2)
                    cv2.circle(frame, (int(a), int(b)), 5, (0, 255, 0), -1)

                    # Update the previous points
                    p0 = good_new.reshape(-1, 1, 2)

            # Update the previous frame
            old_gray = frame_gray.copy()
        else:
            # While waiting for a click, display the live feed and a message
            cv2.putText(frame, "Click a point to track", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)


        cv2.imshow('Point Tracking', frame)


        k = cv2.waitKey(30) & 0xff
        if k == ord('q'):  # Press 'Esc' to exit
            break
        elif k == ord('c'): # Press 'c' to reset and select a new point
            point_selected = False
            p0 = None

cv2.destroyAllWindows()