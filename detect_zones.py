import cv2
import numpy as np


def detect_goals_once(camera_index=0):
    """
    Grab a single frame from the camera and detect red, green, and yellow goals.

    Returns:
        goals (list of dicts)
        frame (np.ndarray)
    """

    FRAME_WIDTH, FRAME_HEIGHT = 640, 480

    # -------------------------
    # Camera setup
    # -------------------------
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE, -7)
    cap.set(cv2.CAP_PROP_AUTO_WB, 0.0)
    cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 3000)

    ret, frame = cap.read()
    cap.release()

    if not ret:
        raise RuntimeError("Failed to grab frame")

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))

    # -------------------------
    # BLACK MASK
    # -------------------------
    color_range = np.max(frame, axis=2) - np.min(frame, axis=2)
    range_thres = np.where(color_range < 20, 255, 0).astype(np.uint8)

    hsv_black = cv2.inRange(hsv, (0, 0, 0), (180, 90, 80))
    black_mask = cv2.bitwise_and(hsv_black, range_thres)
    black_mask = cv2.dilate(black_mask, kernel, iterations=4)
    black_mask = cv2.erode(black_mask, kernel, iterations=1)

    # -------------------------
    # HSV RANGES 
    # -------------------------
    hsv_ranges = {
        "Red":    ((0, 80, 30),  (10, 255, 255)),
        "Green":  ((45, 68, 80), (70, 255, 255)),
        "Yellow": ((30, 90, 100),(45, 255, 255)),
    }

    # -------------------------
    # COLOR MASKS
    # -------------------------
    masks = {}

    for color, (low, high) in hsv_ranges.items():
        if color == "RED":
            mask1 = cv2.inRange(hsv, low, high)
            mask2 = cv2.inRange(
                hsv,
                (170, low[1], low[2]),
                (180, high[1], high[2])
            )
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, low, high)

        mask = cv2.dilate(mask, kernel, iterations=2)
        mask = cv2.erode(mask, kernel, iterations=1)

        masks[color] = cv2.bitwise_and(black_mask, mask)

    # -------------------------
    # PROCESS CONTOURS
    # -------------------------
    goals = []

    for color, mask in masks.items():
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for cnt in contours:
            area = cv2.contourArea(cnt)

            # area rules (yellow is smaller)
            if color == "YELLOW":
                if area < 200:
                    continue
            else:
                if area < 3000:
                    continue

            x, y, w, h = cv2.boundingRect(cnt)

            # ignore 5% top-of-frame detections
            if y < FRAME_HEIGHT * 0.05:
                continue

            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            goals.append({
                "color": color,
                "center_uv": (cx, cy),
                "bbox": (x, y, w, h),
                "area": area
            })

    cv2.imwrite("detected_goals_frame.jpg", frame)
    return goals, frame