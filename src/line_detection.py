import cv2
import numpy as np
import subprocess
from picamera2 import Picamera2

# ----------------------------
# Line detector (unchanged)
# ----------------------------
class LineDetector:
    def __init__(self, frame):
        self.orig = frame
        self.gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def preprocess(self):
        blurred = cv2.GaussianBlur(self.gray, (5, 5), 0)
        return cv2.Canny(blurred, 50, 150)

    def detect_lines(self, edges):
        raw = cv2.HoughLinesP(edges, 1, np.pi/180, 50,
                              minLineLength=50, maxLineGap=10)
        lines = []
        if raw is not None:
            for l in raw:
                lines.append(tuple(l[0]))
        return lines

    def draw_lines(self, lines):
        out = self.orig.copy()
        for x1, y1, x2, y2 in lines:
            cv2.line(out, (x1, y1), (x2, y2), (0, 0, 255), 2)
        return out

# ----------------------------
# Streaming configuration
# ----------------------------
LAPTOP_IP = "192.168.0.242"
PORT = 8554
Width = 640
Height = 480
FPS = 30


# FFmpeg command to stream video
ffmpeg = subprocess.Popen([
    'ffmpeg',
    '-y',
    '-f', 'rawvideo',
    '-pixel_format', 'bgr24',
    '-video_size', f'{Width}x{Height}',
    '-framerate', str(FPS),
    '-i', '-',
    '-c:v', 'libx264',
    '-pix_fmt', 'yuv420p',
    '-preset', 'ultrafast',
    '-f', 'rtsp',
    f'rtsp://{LAPTOP_IP}:{PORT}/live.sdp'
], stdin=subprocess.PIPE)



# ----------------------------
# Camera setup
# ----------------------------
picam = Picamera2()
config = picam.create_preview_configuration(
    main={"size": (Width, Height), "format": "BGR888"}
)
picam.configure(config)
picam.start()


# ----------------------------
# Main loop
# ----------------------------
try:
    while True:
        frame = picam.capture_array()
        detector = LineDetector(frame)
        edges = detector.preprocess()
        lines = detector.detect_lines(edges)
        output_frame = detector.draw_lines(lines)

        # Stream the processed frame
        try:
            ffmpeg.stdin.write(output_frame.tobytes())
        except BrokenPipeError:
            print("FFmpeg pipe closed")
            break
except KeyboardInterrupt:
    print("Stopping...")
finally:
    ffmpeg.stdin.close()
    ffmpeg.wait()
    picam.stop()

