import cv2
import numpy as np

# -----------------------------
# CONFIG / CALIBRATION DATA
# -----------------------------

PTS_IMAGE_PLANE = [
    [373, 223],
    [75, 229],
    [131, 141],
    [165, 89],
    [348, 89],
    [585, 140],
]

PTS_GROUND_PLANE = [
    [0, 36],
    [-12, 36],
    [-12, 48],
    [-12, 60],
    [0, 60],
    [12, 48],
]

# -----------------------------
# CORE LOGIC 
# -----------------------------

def compute_homography(scale=1):
    np_img = np.float32(PTS_IMAGE_PLANE)[:, None, :]
    np_gnd = np.float32(PTS_GROUND_PLANE)[:, None, :] 
    h, _ = cv2.findHomography(np_img, np_gnd)
    return h


def transform_uv_to_xy(h, u, v):
    homogeneous_point = np.array([[u], [v], [1]])
    xy = h @ homogeneous_point
    xy /= xy[2, 0]
    return xy[0, 0], xy[1, 0]
