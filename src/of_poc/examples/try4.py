import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
import math

FPS = 9


# type: ignore
def translation_to_speed(tx, ty, curr_height, fovx_deg, fovy_deg, dtime, image_width, image_height) -> [float, float]:
    half_x_meters = curr_height * math.tan(math.radians(fovx_deg/2))
    half_y_meters = curr_height * math.tan(math.radians(fovy_deg/2))
    vx = (half_x_meters * tx / image_width) / dtime
    vy = (half_y_meters * ty / image_height) / dtime
    return vx, vy


# Read video file
cap = cv2.VideoCapture(
    "/home/user/Projects/robot_localization_ws/of_poc/videos/new.mp4")

# Parameters for Shi-Tomasi corner detection
feature_params = dict(maxCorners=100, qualityLevel=0.3,
                      minDistance=7, blockSize=7)

# Parameters for Lucas-Kanade optical flow
lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(
    cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Capture the first frame and detect corners
ret, old_frame = cap.read()
old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

# Create a mask image for drawing purposes
mask = np.zeros_like(old_frame)

start_time = time.time()
while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Calculate optical flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(
        old_gray, frame_gray, p0, None, **lk_params)

    # Select good points
    good_new = p1[st == 1]
    good_old = p0[st == 1]

    # Compute the transformation matrix (homography) between consecutive frames
    M, _ = cv2.estimateAffinePartial2D(good_old, good_new)
    roll_angle = np.arctan2(M[1, 0], M[0, 0]) * 180 / np.pi
    dtime = time.time() - start_time
    start_time = time.time()
    print(f"{dtime=}")
    # Extract translation in pixels (tx, ty) from the last column of the transformation matrix
    tx = M[0, 2]
    ty = M[1, 2]

    print("Roll Angle:", roll_angle)
    print("Translation (tx, ty):", tx, ty)
    print("Speed (vx, vy): ", translation_to_speed(tx, ty, 80,
          50, 40, dtime, old_gray.shape[0], old_gray.shape[1]))
    # M2, _ = cv2.getPerspectiveTransform(good_new[:4, :], good_old[:4, :])
    # print(f"{M2=}")

    # Draw motion vectors
    for i, (new, old) in enumerate(zip(good_new, good_old)):
        a, b = new.ravel()
        c, d = old.ravel()
        mask = cv2.line(mask, (int(a), int(b)),
                        (int(c), int(d)), (0, 255, 0), 2)
        frame = cv2.circle(frame, (int(a), int(b)), 5, (0, 0, 255), -1)

    title_point = (frame.shape[0]//2, frame.shape[1]//2 - 20)
    cv2.putText(frame, f"roll = {roll_angle}",
                title_point, 1, 2, (255, 0, 0), 2)
    img = cv2.add(frame, mask)

    # Display the resulting frame
    cv2.imshow('Frame', img)

    # Exit if 'q' is pressed
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

    # Update previous frame and points
    old_gray = frame_gray.copy()
    p0 = good_new.reshape(-1, 1, 2)

    time.sleep(float(1/FPS))

cap.release()
cv2.destroyAllWindows()
