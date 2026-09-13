from can_detector_api import detect_cans_once
import cv2

def main():
    # Grab one frame and detect cans
    cans, frame = detect_cans_once()

    for idx, can in enumerate(cans):
        color = can['color']
        orient = can['orient']
        bottom = can['bottom_coords']
        bbox = can['bbox']
        angle = can['angle']
        print(f"Can {idx}: Color={color}, Orient={orient}, Bottom={bottom}, Angle={angle}")

        # Draw info on the frame
        bx, by = bottom
        cv2.circle(frame, (bx, by), 5, (255, 0, 255), -1)
        x, y, w, h = bbox
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(frame, f"{color}", (bx+5, by+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,255), 1)
        cv2.putText(frame, f"{orient}", (bx+5, by+35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

    # Show the frame
    cv2.imshow("Detected Cans", frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
