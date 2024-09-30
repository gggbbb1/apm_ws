import os
import cv2 as cv
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

# invert the original R from direct output of Essential matrix
FOCAL_X = 537
FOCAL_Y = 305


def invert_R(R_mat):
    R_mat_inv = R.from_matrix(R_mat).inv()
    return R_mat_inv.as_euler('zyx', degrees=True)


def show_orb_result(frame):
    orb = cv.ORB_create(edgeThreshold=15, patchSize=31, nlevels=8, fastThreshold=20,
                        scaleFactor=1.2, WTA_K=2, scoreType=cv.ORB_HARRIS_SCORE, firstLevel=0, nfeatures=500)
    # orb = cv2.ORB_create()
    kp2 = orb.detect(frame)
    img2_kp = cv.drawKeypoints(frame, kp2, None, color=(
        0, 255, 0), flags=cv.DrawMatchesFlags_DEFAULT)

    plt.figure()
    plt.imshow(img2_kp)
    plt.show()


# testing code
images_names = [os.path.join("/home/user/Projects/robot_localization_ws/of_poc/videos/yaw", i)
                for i in ["frame1.jpg", "frame2.jpg"]]
img1 = cv.imread(images_names[0])  # self-defined
img2 = cv.imread(images_names[1])

img1 = cv.cvtColor(img1, cv.COLOR_BGR2RGB)
img2 = cv.cvtColor(img2, cv.COLOR_BGR2RGB)

height, width, _ = img1.shape
show_orb_result(img2)
# use ORB detector to do feature matching
orb = cv.ORB_create(edgeThreshold=15, patchSize=31, nlevels=8, fastThreshold=20,
                    scaleFactor=1.2, WTA_K=2, scoreType=cv.ORB_HARRIS_SCORE, firstLevel=0, nfeatures=500)
kp1, des1 = orb.detectAndCompute(img1, None)
kp2, des2 = orb.detectAndCompute(img2, None)

bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
matches = bf.match(des1, des2)
matches = sorted(matches, key=lambda x: x.distance)

# create camera matrix:
camera_mat = np.ndarray((3, 3), dtype=float)
camera_mat.fill(0.0)
camera_mat[0, 0] = FOCAL_X
camera_mat[1, 1] = FOCAL_Y
camera_mat[2, 2] = 1.0
camera_mat[0, 2] = width//2
camera_mat[1, 2] = height//2
print(f"{camera_mat=}")

# extract coordinates of matched keypoints
kpts1 = np.array([kp1[m.trainIdx].pt for m in matches], dtype=np.int32)
kpts2 = np.array([kp2[m.queryIdx].pt for m in matches], dtype=np.int32)
img3 = cv.drawMatches(img1, kp1, img2, kp2,
                      matches[:10], None, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
# trim kpts:
# kpts1 = kpts1[:10]
# kpts2 = kpts2[:10]


# calculate Essential, get Roation matrix
E, mask = cv.findEssentialMat(
    kpts1, kpts2, prob=0.9999, threshold=0.1, cameraMatrix=camera_mat, method=cv.RANSAC)
print(sum(mask), mask.shape)
_, R_est, t_est, mask_pose = cv.recoverPose(E, kpts1, kpts2, cameraMatrix=camera_mat)

# print(R_est, t_est)
# calculate inverted R and origianl R matrix, both displayed in euler angles.
R1_ang = invert_R(R_est)
print("translation: ", t_est)
print("roll, pitch. yaw: ", R1_ang)
plt.imshow(img3), plt.show()
