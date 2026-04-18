#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

import cv2
import numpy as np
import torch
import sys
import os

# Add model repo to path
sys.path.append(os.path.expanduser('~/Individual-project/src/Ultra-Fast-Lane-Detection'))

from model.model import parsingNet


class ultra_fast_lane_node(Node):

    def __init__(self):
        super().__init__('ultra_fast_lane_node')

        self.bridge = CvBridge()

        # Subscriber (camera)
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher (target point)
        self.point_pub = self.create_publisher(Point, '/target_point', 10)

        # Load model
        self.model = parsingNet(pretrained=False, backbone='18', cls_dim=(101, 56, 4))
        weights_path = os.path.expanduser('~/Individual-project/src/Ultra-Fast-Lane-Detection/weights/tusimple_18.pth')
        self.model.load_state_dict(torch.load(weights_path, map_location='cpu'))
        self.model.eval()

        self.get_logger().info("UltraFast Lane Node Started")

    def run_model(self, frame):
        img = cv2.resize(frame, (800, 288))
        img = img[:, :, ::-1]
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        img = torch.tensor(img).unsqueeze(0)

        with torch.no_grad():
            output = self.model(img)

        return output

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Run model
        output = self.run_model(frame)

        # ⚠️ TEMP: fake center (replace later with real lane extraction)
        h, w = frame.shape[:2]
        target_x = w // 2
        target_y = int(h * 0.7)

        # Publish target
        point_msg = Point()
        point_msg.x = float(target_x)
        point_msg.y = float(target_y)
        point_msg.z = 0.0

        self.point_pub.publish(point_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ultra_fast_lane_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()