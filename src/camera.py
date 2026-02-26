from picamera2 import Picamera2
import cv2

def run_camera(window_name="Camera Feed", quit_key='q'):
    picam2 = Picamera2()

    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "BGR888"}
    )
    picam2.configure(config)
    picam2.start()

    try:
        while True:
            frame = picam2.capture_array()
            cv2.imshow(window_name, frame)

            if cv2.waitKey(1) & 0xFF == ord(quit_key):
                break
    finally:
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    run_camera()

