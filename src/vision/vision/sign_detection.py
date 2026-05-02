#!/usr/bin/env python3

import rclpy
from rclpy.time import Time
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from messages.msg import Sign
import numpy as np
import cv2
from ultralytics import YOLO
from cv_bridge import CvBridge


class TrafficSignDetector(Node):

    def __init__(self):
        super().__init__('traffic_sign_detector')
        self.bridge = CvBridge()

        # Load model
        self.model = YOLO("models/traffic_sign_detector.pt")

        # Publisher for detected sign
        self.pub = self.create_publisher(Sign, "/detected_sign", 10)
        self.pub.publish(Sign(label="None", confidence=0.0))  # Initial state publish

        #publisher for sign image
        self.image_pub = self.create_publisher(CompressedImage, "/vision/detected_sign_image/compressed", 10)

        # Subscriber to camera
        self.sub = self.create_subscription(
            CompressedImage,
            "/camera/image_raw/compressed",
            self.image_callback,
            10
        )

        self.get_logger().info("Traffic Sign Detector Node Started")

    def image_callback(self, msg):

        t_capture = Time.from_msg(msg.header.stamp)
        # Convert compressed image → OpenCV frame
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warn("Failed to decode image")
            return

        # Run YOLO inference

        results = self.model(frame, verbose=False)

        detected_sign = None
        best_conf = 0.0

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.model.names[cls]

                # keep best detection only
                if conf > best_conf:
                    best_conf = conf
                    detected_sign = label

                # optional debug box draw
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {conf:.2f}",
                            (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            2)

        # Publish result
        if detected_sign:
            msg_out = Sign()
            msg_out.header.stamp = msg.header.stamp  # Use the same timestamp as the input image
            msg_out.label = detected_sign
            msg_out.confidence = best_conf
            self.pub.publish(msg_out)

            self.get_logger().info(f"Detected: {detected_sign} ({best_conf:.2f})")

           
        else:
            self.get_logger().info("No sign detected")
            #self.pub.publish(Sign(label="None", confidence=0.0))  # Publish "None" if no sign detected
        # Publish detected sign image
        img_msg = CompressedImage()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.format = "jpeg"
        img_msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()
        self.image_pub.publish(img_msg)
        
        t_now = self.get_clock().now()
        delay = (t_now - t_capture).nanoseconds * 1e-9  
        self.get_logger().info(f"Processing time: {delay:.3f} seconds")
    

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()