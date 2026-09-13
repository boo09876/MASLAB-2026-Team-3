import cv2
import numpy as np


def detect_goals_2(camera_index=0):
    # -------------------------
    # Camera setup
    # -------------------------
    FRAME_WIDTH, FRAME_HEIGHT = 640, 480

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

    # -------------------------
    # Convert to HSV
    # -------------------------
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # -------------------------
    # BLACK MASK
    # -------------------------
    color_range = np.max(frame, axis=2) - np.min(frame, axis=2)
    range_thres = np.where(color_range < 20, 255, 0).astype(np.uint8)

    hsv_black = cv2.inRange(hsv, (0, 0, 0), (180, 90, 150))
    black_mask = cv2.bitwise_and(hsv_black, range_thres)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
    black_mask = cv2.dilate(black_mask, kernel, iterations=3)
    black_mask = cv2.erode(black_mask, kernel, iterations=1)

    # -------------------------
    # COLOR MASKS
    # -------------------------
    red1 = cv2.inRange(hsv, (0, 80, 30), (10, 255, 255))
    red2 = cv2.inRange(hsv, (170, 100, 40), (180, 255, 255))
    red_mask = cv2.bitwise_or(red1, red2)

    green_mask = cv2.inRange(hsv, (40, 40, 100), (60, 100, 255))
    yellow_mask = cv2.inRange(hsv, (30, 90, 100), (45, 150, 255))
    
    color_masks = [red_mask, green_mask, yellow_mask]
    for i in range(len(color_masks)):
        color_masks[i] = cv2.dilate(color_masks[i], kernel, iterations=2)
        color_masks[i] = cv2.erode(color_masks[i], kernel, iterations=1)
    
    red_mask, green_mask, yellow_mask = color_masks


    # -------------------------
    # COMBINED GOAL MASKS
    # -------------------------
    red_goal = cv2.bitwise_and(black_mask, red_mask)
    green_goal = cv2.bitwise_and(black_mask, green_mask)
    yellow_goal = cv2.bitwise_and(black_mask, yellow_mask)

    # cv2.imshow("Black Mask", black_mask)
    # cv2.imshow("Raw Yellow Mask", yellow_mask)
    # cv2.imshow("Black and Yellow", yellow_goal)
    
    goals = []
    
    # -------------------------
    # PROCESS EACH COLOR
    # -------------------------
    def process_goal(mask, label):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 2000:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            # aspect = w / float(h)
            # if aspect < 0.7 or aspect > 1.3:
            #     continue

            # ---- CENTROID USING MOMENTS ----
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            goals.append({
                "color": label,
                "center_uv": (cx, cy),
                "bbox": (x, y, w, h),
                "area": area
            })

            # ---- DEBUG DRAW (BLACK / WHITE ONLY) ----
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 0), 2)
            cv2.circle(frame, (cx, cy), 6, (255, 255, 255), -1)
            cv2.putText(
                frame,
                f"Zone: {label}",
                (x, y-8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

    process_goal(red_goal, "RED")
    process_goal(green_goal, "GREEN")
    process_goal(yellow_goal, "YELLOW")

    # -------------------------
    # DEBUG WINDOWS
    # -------------------------
    # cv2.imshow("Detected Goals", frame)
    # cv2.imshow("Black Mask", black_mask)
    # cv2.imshow("Red Goal Mask", red_goal)
    # cv2.imshow("Green Goal Mask", green_goal)
    # cv2.imshow("Yellow Goal Mask", yellow_goal)

    # while True:
    #     key = cv2.waitKey(1) & 0xFF
    #     if key == ord('q'):
    #         break
    # cv2.destroyAllWindows()

    return goals, frame


# -------------------------
# RUN
# -------------------------
if __name__ == "__main__":
    goals, frame = detect_goals_2()
    print(goals)
