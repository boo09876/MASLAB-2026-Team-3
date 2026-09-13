import cv2
import numpy as np

def is_blue_close(camera_index=0, area_threshold=37000):
    """
    Returns True if a large blue region (like perimeter tape) is detected.
    
    camera_index: which camera to use
    area_threshold: minimum area (in pixels) to consider "large"
    """
    FRAME_WIDTH, FRAME_HEIGHT = 640, 480

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    ret, frame = cap.read()
    cap.release()
    if not ret:
        raise RuntimeError("Failed to grab frame")

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # HSV range for blue tape
    blue_lower = np.array([100, 150, 50])
    blue_upper = np.array([130, 255, 255])

    mask = cv2.inRange(hsv, blue_lower, blue_upper)

    # Morphological closing to fill gaps
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Check if any contour is large enough
    for c in contours:
        if cv2.contourArea(c) > area_threshold:
            return True

    return False


# # -------------------------
# # TEST
# # -------------------------
# if __name__ == "__main__":
#     if is_blue_close():
#         print("Large blue detected!")
#     else:
#         print("No large blue detected.")
