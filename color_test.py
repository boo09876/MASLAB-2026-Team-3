import cv2  # OpenCV version 2
import numpy as np

IMAGE_WINDOW_NAME = "color"

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Change to your appropriate camera
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
cap.set(cv2.CAP_PROP_EXPOSURE, 250)
cap.set(cv2.CAP_PROP_AUTO_WB, 0)
cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 3000) #3000

# Mouse click event listener
def mouse_event_listener(event, u, v, flags, param):
    # For left mouse click
    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_event_listener.clicked_point = (u, v)


mouse_event_listener.clicked_point = None


# Take a picture and show frame
ret = False
while not ret:
    ret, frame = cap.read()
cv2.imshow(IMAGE_WINDOW_NAME, frame)

# Set click callback
cv2.setMouseCallback(IMAGE_WINDOW_NAME, mouse_event_listener)

# Wait for q or close to quit
while True:
    ret, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if mouse_event_listener.clicked_point is not None:
        u, v = mouse_event_listener.clicked_point
        # Draw red circle
        cv2.circle(
            frame,
            (u, v),
            10,
            (0, 0, 255),
            2,
        )
        # Print BGR
        cv2.putText(
            frame,
            f"bgr{frame[v][u]}",
            (u - 100, v - 20),
            cv2.FONT_HERSHEY_PLAIN,
            2,
            (0, 0, 255),
            2,
        )
        # Print HSV
        cv2.putText(
            frame,
            f"hsv{hsv_frame[v][u]}",
            (u - 100, v - 60),
            cv2.FONT_HERSHEY_PLAIN,
            2,
            (0, 0, 255),
            2,
        )
        print("HSV: ", f"hsv{hsv_frame[v][u]}")

    cv2.imshow(IMAGE_WINDOW_NAME, frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
    try:
        cv2.getWindowProperty(IMAGE_WINDOW_NAME, 0)
    except cv2.error:
        break
cv2.destroyAllWindows()
cap.release()