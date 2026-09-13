import cv2
import numpy as np

IMAGE_WINDOW_NAME = "estimation"

# TODO: Change to your appropriate image points
# PTS_IMAGE_PLANE = [
#     [256, 439],
#     [351, 364],
#     [190, 369],
#     [413, 438],
#     [378, 392],
#     [175, 405],
#     [271, 367],
# ]
PTS_IMAGE_PLANE = [
    [325, 304],
    [328, 173],
    [330, 108],
    [95, 107],
    [564, 103],
    [329, 64],
]

# TODO: Change to your appropriate real-world points
# PTS_GROUND_PLANE = [
#     [0.75, -0.75],
#     [4.5, 1.5],
#     [4.5, -4],
#     [0.75, 3],
#     [3, 2.25],
#     [2.25, -3],
#     [4.5, -0.75],
# ]

PTS_GROUND_PLANE = [
    [0, 0],
    [0, 12],
    [0, 24],
    [-12, 24],
    [12, 24],
    [0, 36],
]


# TODO: Change to your appropriate camera
cap = cv2.VideoCapture(0)

# Take a picture and show frame
ret = False
while not ret:
    ret, frame = cap.read()
    cap.release()
cv2.imshow(IMAGE_WINDOW_NAME, frame)


def transform_uv_to_xy(h, u, v):
    """
    u and v are pixel coordinates.
    The top left pixel is the origin, u axis increases to right, and v axis
    increases down.

    Returns a normal non-np 1x2 matrix of xy displacement vector from the
    camera to the point on the ground plane.
    Camera points along positive x axis and y axis increases to the left of
    the camera.

    Units are in whichever unit h was calculated in.
    """
    homogeneous_point = np.array([[u], [v], [1]])
    xy = np.dot(h, homogeneous_point)
    scaling_factor = 1.0 / xy[2, 0]
    homogeneous_xy = xy * scaling_factor
    x = homogeneous_xy[0, 0]
    y = homogeneous_xy[1, 0]
    return x, y + 20  # Adjust for camera offset, change later


np_pts_ground = np.array(PTS_GROUND_PLANE)
np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :]) 

np_pts_image = np.array(PTS_IMAGE_PLANE)
np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

h, err = cv2.findHomography(np_pts_image, np_pts_ground)
cv2.circle(frame, (340, 252), 5, (0,0,0), -1)
cv2.imshow(IMAGE_WINDOW_NAME, frame)

# Mouse click event listener
def mouse_event_listener(event, u, v, flags, param):
    frame = param
    # For left mouse click
    if event == cv2.EVENT_LBUTTONDOWN:
        # Transform to x y (world coordinate)
        x, y = transform_uv_to_xy(h, u, v)
        print(f"x: {x}, y: {y:}")  # Print world coordinate
        test_frame = cv2.circle(
            frame.copy(), (u, v), 10, (0, 0, 255), 2
        )  # Draw red circle

        # Print world coordinate on picture
        cv2.putText(
            test_frame,
            f"x: {x:.2f} y: {y:.2f}",
            (u - 120, v - 20),
            cv2.FONT_HERSHEY_PLAIN,
            2,
            (0, 0, 255),
            2,
        )
        cv2.imshow(IMAGE_WINDOW_NAME, test_frame)


# Set click callback
cv2.setMouseCallback(IMAGE_WINDOW_NAME, mouse_event_listener, frame)

# Wait for q or close to quit
while True:
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
    try:
        cv2.getWindowProperty(IMAGE_WINDOW_NAME, 0)
    except cv2.error:
        break
cv2.destroyAllWindows()
