import cv2
import numpy as np
from collections import deque
from camera_utils import (
    make_kernel, clean_mask, contour_color_fractions, decide_color,
    orientation_from_min_area_rect
)
from get_rw_coord import transform_uv_to_xy, compute_homography


def detect_cans_once(camera_index=0, img_num = 0, show_debug=False):

    
    """
    Grab a single frame and return a list of detected cans with all info.
    
    Returns:
        detected_cans: list of dicts, each dict has:
            - 'color': str
            - 'orient': str
            - 'bottom_coords': (x, y)
            - 'bbox': (x, y, w, h)
            - 'angle': float
            - 'stats': (red_frac, green_frac, yellow_frac, ratio, theta)
            - 'contour': numpy.ndarray

        frame: the captured image 
    """
    MIN_COLOR_FRACTION = 0.90
    BLACK_RATIO_REJECT = 0.10

    # Camera setup
    FRAME_WIDTH, FRAME_HEIGHT = 640, 480

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv2.CAP_PROP_EXPOSURE, 200)
    cap.set(cv2.CAP_PROP_AUTO_WB, 0.0)
    cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 3000)

    ret, frame = cap.read()
    cap.release()
    if not ret:
        raise RuntimeError("Cannot grab frame")

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Thresholds 
    lower_r1, upper_r1 = np.array([0, 80, 30]), np.array([10, 255, 255])
    lower_r2, upper_r2 = np.array([170, 80, 30]), np.array([180, 255, 255])
    lower_g, upper_g = np.array([40, 70, 60]), np.array([80, 255, 255])
    lower_y, upper_y = np.array([20, 100, 100]), np.array([40, 255, 255])
    lower_black, upper_black = np.array([0, 0, 0]), np.array([180, 80, 150]) 

    # Masks
    red_mask = cv2.inRange(hsv, lower_r1, upper_r1) | cv2.inRange(hsv, lower_r2, upper_r2)
    green_mask = cv2.inRange(hsv, lower_g, upper_g)
    yellow_mask = cv2.inRange(hsv, lower_y, upper_y)
    black_mask = cv2.inRange(hsv, lower_black, upper_black)

    # Clean masks
    kernel = make_kernel(5)
    red_mask = clean_mask(red_mask, kernel)
    green_mask = clean_mask(green_mask, kernel)
    yellow_mask = clean_mask(yellow_mask, kernel)
    

    # Union mask for candidate detection
    union_mask = cv2.bitwise_or(red_mask, cv2.bitwise_or(green_mask, yellow_mask))

    contours, _ = cv2.findContours(union_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f"Cans: Found {len(contours)} contours")

    detected_cans = []

    for cnt in contours:
        # print("entered contour loop")
        x, y, w, h = cv2.boundingRect(cnt)
        if w < 20 or h < 20:
            continue
        # print("passed size filter")
        area = cv2.contourArea(cnt)
        if area < 2000 or area > 250000:
            continue
        # print("passed area filter")

        # looking at black ratio inside and outside the contour -------
        pad = 10
        x0 = max(0, x - pad)
        y0 = max(0, y - pad)
        x1 = min(frame.shape[1], x + w + pad)
        y1 = min(frame.shape[0], y + h + pad)

        # mask for the contour itself
        contour_mask = np.zeros_like(black_mask)
        cv2.drawContours(contour_mask, [cnt], -1, 255, -1)
        # cv2.imshow("Contour Mask", contour_mask)

        # black inside contour
        # cv2.imshow("Black Mask", black_mask)
        black_inside = cv2.bitwise_and(black_mask, contour_mask)
        black_ratio_inside = cv2.countNonZero(black_inside) / max(cv2.countNonZero(contour_mask), 1)
        # cv2.imshow("Black Inside", black_inside)
        print(f"Black ratio inside contour: {black_ratio_inside:.2f}")

        # black outside contour
        roi_black_outside = black_mask[y0:y1, x0:x1].copy()
        roi_contour_mask = contour_mask[y0:y1, x0:x1]
        roi_black_outside = cv2.bitwise_and(roi_black_outside, cv2.bitwise_not(roi_contour_mask))
        black_ratio_outside = cv2.countNonZero(roi_black_outside) / roi_black_outside.size
        # cv2.imshow("Black Outside", roi_black_outside)
        print(f"Black ratio outside contour: {black_ratio_outside:.2f}")

        if black_ratio_inside > BLACK_RATIO_REJECT or black_ratio_outside > BLACK_RATIO_REJECT:
            continue
            

        red_frac, green_frac, yellow_frac = contour_color_fractions(cnt, red_mask, green_mask, yellow_mask)

        color_label = "UNKNOWN"
        best_frac = max(red_frac, green_frac, yellow_frac)
        if best_frac >= MIN_COLOR_FRACTION:
            color_label = decide_color(red_frac, green_frac, yellow_frac)

        rect = cv2.minAreaRect(cnt)
        orient_label, ratio, theta = orientation_from_min_area_rect(rect)

        bottom_coords = ((x+(x+w))//2, y + h)

        detected_cans.append({
            'color': color_label,
            'orient': orient_label,
            'bottom_coords': bottom_coords,
            'bbox': (x, y, w, h),
            'angle': theta,
            'stats': (red_frac, green_frac, yellow_frac, ratio, theta),
            # 'contour': cnt
        })

        # DRAW FOR DEBUG
        debug_color = (0, 0, 0)
        cv2.drawContours(frame, [cnt], -1, debug_color, 2)  # contour in blue
        cv2.rectangle(frame, (x, y), (x + w, y + h), debug_color, 2)
        bx, by = bottom_coords
        cv2.circle(frame, (bx, by), 5, debug_color, -1)
        cv2.putText(frame, f"Can: {color_label}", (bx+5, by+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, debug_color, 1)
        cv2.putText(frame, f"{orient_label}", (bx+5, by+35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, debug_color, 1)
        cv2.putText(frame, f"R:{red_frac:.2f} G:{green_frac:.2f} Y:{yellow_frac:.2f}", 
                    (bx+5, by+50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, debug_color, 1)
        
    
    # Show mask and frame
    if show_debug:
        cv2.imshow("Union Mask", union_mask)
        cv2.imshow("Detected Cans", frame)

        key = cv2.waitKey(0) & 0xFF
        if key == ord('q'):
            cv2.destroyAllWindows()

        
    cv2.imwrite("detected_can" + str(img_num) + ".jpg", frame)

    return detected_cans, frame

    
# def detect_goals_2():
#     # Camera setup
#     FRAME_WIDTH, FRAME_HEIGHT = 640, 480

#     cap = cv2.VideoCapture(0)
#     if not cap.isOpened():
#         raise RuntimeError("Cannot open camera")
#     cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
#     cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

#     cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Set to manual exposure mode with 0.25 "magic number"
#     cap.set(cv2.CAP_PROP_EXPOSURE, -7)  # Set exposure time to 2^-7 = 1/128 second
#     cap.set(cv2.CAP_PROP_AUTO_WB, 0.0) # Disable auto white balance
#     cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 3000)

#     ret, frame = cap.read()

#     ### Black
#     color_range = np.max(frame[:, :, 1:], axis=2) - np.min(frame[:, :, 1:], axis=2)
#     range_thres = np.where(color_range < 10, 255, 0).astype(np.uint8)
#     hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#     hsv_thres = cv2.inRange(hsv_frame, (0, 0, 0), (180, 255, 70))

#     black_thres = cv2.inRange(hsv_frame, (0, 0, 0), (180, 120, 90)) ### ADDED ###
#     kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
#     black_thres = cv2.dilate(black_thres, kernel, iterations=2)
#     black_thres = cv2.erode(black_thres, kernel, iterations=1)

#     red_thres1 = cv2.inRange(hsv_frame, (0, 80, 30), (10, 255, 255))
#     red_thres2 = cv2.inRange(hsv_frame, (170, 100, 40), (180, 255, 255))

#     red_thres = cv2.bitwise_or(red_thres1, red_thres2)
#     red_thres = cv2.dilate(red_thres, kernel, iterations=2)
#     red_thres = cv2.erode(red_thres, kernel, iterations=1)

#     green_thres = cv2.inRange(hsv_frame, (50, 60, 80), (65, 255, 255))
#     green_thres = cv2.dilate(green_thres, kernel, iterations=2)
#     green_thres = cv2.erode(green_thres, kernel, iterations=1)

#     yellow_thres = cv2.inRange(hsv_frame, (20, 100, 100), (45, 255, 255))
#     yellow_thres = cv2.dilate(yellow_thres, kernel, iterations=2)
#     yellow_thres = cv2.erode(yellow_thres, kernel, iterations=1)

#     green_goal_thres = cv2.bitwise_and(black_thres, green_thres)
#     red_goal_thres = cv2.bitwise_and(black_thres, red_thres)
#     yellow_goal_thres = cv2.bitwise_and(black_thres, yellow_thres)

#     cv2.imshow("Detected Goals", frame)
#     # cv2.imshow("Black Mask", black_thres)
#     # cv2.imshow("Green Mask", green_thres)
#     cv2.imshow("Green Goal Mask", green_goal_thres)
#     # cv2.imshow("Red Mask", red_thres)
#     cv2.imshow("Red Goal Mask", red_goal_thres)

#     cv2.waitKey(0)
#     cv2.destroyAllWindows()


# detected_cans, _ = detect_cans_once()
# print(detected_cans)

if __name__ == "__main__":
    detect_cans_once(camera_index=0, img_num=0, show_debug=False)
