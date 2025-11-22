import cv2

def run_camera(device=0, window_name='Camera Feed', quit_key='q'):
    """Open camera and show feed until quit_key is pressed."""
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        print("Error: Could not open video.")
        return

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame.")
                break

            cv2.imshow(window_name, frame)
            if cv2.waitKey(1) & 0xFF == ord(quit_key):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    run_camera()
