import cv2
import numpy as np

def test_blue_area(camera_index=0):
    FRAME_WIDTH, FRAME_HEIGHT = 640, 480

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # HSV range for blue tape (adjust if needed)
        blue_lower = np.array([100, 150, 50])
        blue_upper = np.array([130, 255, 255])

        mask = cv2.inRange(hsv, blue_lower, blue_upper)

        # Morphological closing to fill gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find largest area
        largest_area = 0
        for c in contours:
            area = cv2.contourArea(c)
            if area > largest_area:
                largest_area = area

        print(f"Largest blue area: {largest_area}")

        # Show mask for visualization
        cv2.imshow("Blue Mask", mask)
        cv2.imshow("Camera", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    test_blue_area()
