import cv2
import numpy as np
from collections import deque
from camera_utils import (
    make_kernel, clean_mask, contour_color_fractions, decide_color,
    orientation_from_min_area_rect
)



def detect_cans_live(camera_index=0):
    MIN_COLOR_FRACTION = 0.90
    BLACK_RATIO_REJECT = 0.10

    FRAME_WIDTH, FRAME_HEIGHT = 640, 480

    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE, 200)
    cap.set(cv2.CAP_PROP_AUTO_WB, 0.0)
    cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 3500)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Thresholds
        lower_r1, upper_r1 = np.array([0, 80, 30]), np.array([10, 255, 255])
        lower_r2, upper_r2 = np.array([170, 80, 30]), np.array([180, 255, 255])
        lower_g, upper_g = np.array([40, 70, 60]), np.array([80, 255, 255])
        lower_y, upper_y = np.array([20, 100, 100]), np.array([40, 255, 255])
        lower_black, upper_black = np.array([0, 0, 0]), np.array([180, 80, 100]) #150

        red_mask = cv2.inRange(hsv, lower_r1, upper_r1) | cv2.inRange(hsv, lower_r2, upper_r2)
        green_mask = cv2.inRange(hsv, lower_g, upper_g)
        yellow_mask = cv2.inRange(hsv, lower_y, upper_y)
        black_mask = cv2.inRange(hsv, lower_black, upper_black)

        kernel = make_kernel(5)
        red_mask = clean_mask(red_mask, kernel)
        green_mask = clean_mask(green_mask, kernel)
        yellow_mask = clean_mask(yellow_mask, kernel)

        union_mask = cv2.bitwise_or(red_mask, cv2.bitwise_or(green_mask, yellow_mask))

        contours, _ = cv2.findContours(union_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(f"Cans: Found {len(contours)} contours")

        detected_cans = []
        
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if w < 20 or h < 20:
                continue

            area = cv2.contourArea(cnt)
            if area < 2000 or area > 250000:
                continue

            # looking at black ratio inside and outside the contour -------
            pad = 10
            x0 = max(0, x - pad)
            y0 = max(0, y - pad)
            x1 = min(frame.shape[1], x + w + pad)
            y1 = min(frame.shape[0], y + h + pad)

            # mask for the contour itself
            contour_mask = np.zeros_like(black_mask)
            cv2.drawContours(contour_mask, [cnt], -1, 255, -1)
            cv2.imshow("Contour Mask", contour_mask)

            # black inside contour
            cv2.imshow("Black Mask", black_mask)
            black_inside = cv2.bitwise_and(black_mask, contour_mask)
            black_ratio_inside = cv2.countNonZero(black_inside) / max(cv2.countNonZero(contour_mask), 1)
            cv2.imshow("Black Inside", black_inside)
            print(f"Black ratio inside contour: {black_ratio_inside:.2f}")

            # black outside contour
            roi_black_outside = black_mask[y0:y1, x0:x1].copy()
            roi_contour_mask = contour_mask[y0:y1, x0:x1]
            roi_black_outside = cv2.bitwise_and(roi_black_outside, cv2.bitwise_not(roi_contour_mask))
            black_ratio_outside = cv2.countNonZero(roi_black_outside) / roi_black_outside.size
            cv2.imshow("Black Outside", roi_black_outside)
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
            bx, by = bottom_coords

            detected_cans.append({
                'color': color_label,
                'orient': orient_label,
                'bottom_coords': bottom_coords,
                'bbox': (x, y, w, h),
                'angle': theta,
                'stats': (red_frac, green_frac, yellow_frac, ratio, theta),
                'contour': cnt
            })
            debug_color = (0, 0, 0)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 0), 2)
            cv2.putText(frame, f"Can: {color_label}", (bx+5, by+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, debug_color, 1)

        # LIVE WINDOWS
        cv2.imshow("Detected Cans", frame)
        cv2.imshow("Union Mask", union_mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    detect_cans_live()