import cv2
import numpy as np

def nothing(x):
    pass

def detect_goals_2(camera_index=0):
    # -------------------------
    # Camera setup
    # -------------------------
    FRAME_WIDTH, FRAME_HEIGHT = 640, 480

    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv2.CAP_PROP_EXPOSURE, 250)
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)
    cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 3500) #3000

    # -------------------------
    # CREATE HSV TRACKBARS
    # -------------------------
    cv2.namedWindow("HSV Tuner")
    colors = ["RED", "GREEN", "YELLOW"]
    for c in colors:
        for t in ["H_min", "H_max","S_min", "S_max", "V_min", "V_max"]:
            cv2.createTrackbar(f"{c}_{t}", "HSV Tuner", 0, 255, nothing)
    # defaults
    cv2.setTrackbarPos("RED_H_min", "HSV Tuner", 0)
    cv2.setTrackbarPos("RED_H_max", "HSV Tuner", 10)
    cv2.setTrackbarPos("RED_S_min", "HSV Tuner", 80)
    cv2.setTrackbarPos("RED_S_max", "HSV Tuner", 255)
    cv2.setTrackbarPos("RED_V_min", "HSV Tuner", 30)
    cv2.setTrackbarPos("RED_V_max", "HSV Tuner", 255)

    cv2.setTrackbarPos("GREEN_H_min", "HSV Tuner", 45)
    cv2.setTrackbarPos("GREEN_H_max", "HSV Tuner", 70)
    cv2.setTrackbarPos("GREEN_S_min", "HSV Tuner", 68)
    cv2.setTrackbarPos("GREEN_S_max", "HSV Tuner", 255)
    cv2.setTrackbarPos("GREEN_V_min", "HSV Tuner", 80)
    cv2.setTrackbarPos("GREEN_V_max", "HSV Tuner", 255)

    cv2.setTrackbarPos("YELLOW_H_min", "HSV Tuner", 30)
    cv2.setTrackbarPos("YELLOW_H_max", "HSV Tuner", 45)
    cv2.setTrackbarPos("YELLOW_S_min", "HSV Tuner", 90)
    cv2.setTrackbarPos("YELLOW_S_max", "HSV Tuner", 255)
    cv2.setTrackbarPos("YELLOW_V_min", "HSV Tuner", 100)
    cv2.setTrackbarPos("YELLOW_V_max", "HSV Tuner", 255)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    last_goals = []
    
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # -------------------------
        # BLACK MASK
        # -------------------------
        color_range = np.max(frame[:, :, 1:], axis=2) - np.min(frame[:, :, 1:], axis=2)
        range_thres = np.where(color_range < 10, 255, 0).astype(np.uint8)
        hsv_black = cv2.inRange(hsv, (0, 0, 0), (180, 90, 120))
        black_mask = cv2.bitwise_and(hsv_black, range_thres)
        black_mask = cv2.dilate(black_mask, kernel, iterations=3)
        black_mask = cv2.erode(black_mask, kernel, iterations=1)
        cv2.imshow("Black Mask", black_mask)

        # -------------------------
        # COLOR MASKS USING TRACKBARS
        # -------------------------
        masks = {}
        for c in colors:
            h_min = cv2.getTrackbarPos(f"{c}_H_min", "HSV Tuner")
            s_min = cv2.getTrackbarPos(f"{c}_S_min", "HSV Tuner")
            v_min = cv2.getTrackbarPos(f"{c}_V_min", "HSV Tuner")
            h_max = cv2.getTrackbarPos(f"{c}_H_max", "HSV Tuner")
            s_max = cv2.getTrackbarPos(f"{c}_S_max", "HSV Tuner")
            v_max = cv2.getTrackbarPos(f"{c}_V_max", "HSV Tuner")

            if c == "RED":
                mask1 = cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))
                mask2 = cv2.inRange(hsv, (170, s_min, v_min), (180, s_max, v_max))
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))

            mask = cv2.dilate(mask, kernel, iterations=2)
            mask = cv2.erode(mask, kernel, iterations=1)

            # BITWISE AND WITH BLACK MASK
            masks[c] = cv2.bitwise_and(black_mask, mask)

        # -------------------------
        # PROCESS CONTOURS
        # -------------------------
        goals = []
        for color, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                print("Color: ", color, " Area: ", area)
                if area < 2000 and color!= "YELLOW":
                    continue
                elif color == "YELLOW" and area < 200:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                # if x < FRAME_HEIGHT * 0.05:  # IGNORE CONTOURS FAR UP (top 10%)
                #     continue

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

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 0), 2)
                cv2.circle(frame, (cx, cy), 6, (255, 255, 255), -1)
                cv2.putText(frame, f"Zone: {color}", (x, y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # # Only print if goals changed    
            # if goals != last_goals:
            #     print(goals)
            #     last_goals = goals.copy()

            # -------------------------
            # SHOW RESULTS
            # -------------------------
            cv2.imshow("Detected Goals", frame)
            # cv2.imshow("Black Mask", black_mask)
            cv2.imshow("Red Goal", masks["RED"])
            cv2.imshow("Green Goal", masks["GREEN"])
            cv2.imshow("Yellow Goal", masks["YELLOW"])

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return goals, frame


if __name__ == "__main__":
    goals, frame = detect_goals_2()
    print(goals)
