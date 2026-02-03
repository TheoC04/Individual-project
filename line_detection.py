import cv2
import numpy as np
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
# GStreamer pipeline
# ----------------------------
LAPTOP_IP = "192.168.0.242"
PORT = 5000

gst_pipeline = (
    f'appsrc is-live=true block=true format=time '
    f'caps=video/x-raw,format=BGR,width=640,height=480,framerate=30/1 ! '
    f'videoconvert ! '
    f'x264enc tune=zerolatency bitrate=1500 speed-preset=ultrafast key-int-max=30 ! '
    f'rtph264pay config-interval=1 pt=96 ! '
    f'udpsink host={LAPTOP_IP} port={PORT} sync=false'
)


# ----------------------------
# Camera setup
# ----------------------------
picam = Picamera2()
config = picam.create_preview_configuration(
    main={"size": (640, 480), "format": "BGR888"}
)
picam.configure(config)
picam.start()

out = cv2.VideoWriter(
    gst_pipeline,
    cv2.CAP_GSTREAMER,
    0,
    30,
    (640, 480),
    True
)

if not out.isOpened():
    print("❌ GStreamer pipeline FAILED to open")
    exit(1)
else:
    print("✅ GStreamer pipeline opened")
# ----------------------------
# Main loop
# ----------------------------
while True:
    frame = picam.capture_array()
    det = LineDetector(frame)
    edges = det.preprocess()
    lines = det.detect_lines(edges)
    output = det.draw_lines(lines)
    #print (f"Detected {len(lines)} lines")
    out.write(output)
