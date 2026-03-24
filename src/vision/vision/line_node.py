import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import geometry_msgs.msg


#TODO: refactor this into a separate class that just does line detection, and then the node just calls it and publishes results. This way we can also use it in the lane follower node without duplicating code.
# The LineDetector class can have methods for preprocessing, line detection, curve fitting, and ROI application. The node just handles ROS communication and calls these methods.
# This also makes it easier to test the line detection logic separately from ROS, and allows for more modular code. The node can be renamed to LineDetectionNode to reflect its purpose.

#TODO: add curve detection that finds the edges of the lane and plots the middle curve. This can be done by scanning horizontally across the image at different heights, finding the left and right lane edges, and fitting a polynomial to those points to get the curve. The curve can then be drawn on the output image for visualization.

#TODO: Bird's eye view transformation to get a top-down view of the lane, which can make line detection more robust. This involves defining a perspective transform based on known points in the image and applying it to warp the image to a bird's eye view. Then line detection can be performed on this warped image, and the results can be transformed back to the original perspective for visualization and target point selection.

class LineDetector:
    def __init__(self, frame):
        self.orig = frame
        self.gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def preprocess(self):
        blurred = cv2.GaussianBlur(self.gray, (5, 5), 0)
        return cv2.Canny(blurred, 50, 150)

    def detect_lines(self, edges):
        
        raw = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180,
            50,
            minLineLength=50,
            maxLineGap=10
        )

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
    
    def apply_roi(self, image):
        h, w = image.shape[:2]

        # Create an empty mask (same size as input)
        mask = np.zeros_like(image)

        # Define trapezoid (tweak these values for your camera view)
        polygon = np.array([[
            (int(0 * w), 0.9 * h),          # bottom left (x,y)
            (int(1 * w), 0.9 * h),          # bottom right
            (int(0.6 * w), int(0.4*h)), # top right
            (int(0.4 * w), int(0.4*h))  # top left
        ]], np.int32)

        # Fill the ROI area
        if len(image.shape) == 2:
            cv2.fillPoly(mask, polygon, 255)  # grayscale
        else:
            cv2.fillPoly(mask, polygon, (255,)*image.shape[2])  # color

        # Apply mask
        masked = cv2.bitwise_and(image, mask)

        return masked
    
    def detect_curve(self, image):
        # Placeholder for curve detection logic
        # This could involve fitting a polynomial to the detected line points
        # and calculating curvature based on the fitted curve.

        h, w = image.shape[:2]

        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Yellow + white masks
        yellow = cv2.inRange(hsv, (20, 100, 100), (30, 255, 255))
        white  = cv2.inRange(hsv, (0, 0, 200), (180, 50, 255))

        mask = cv2.bitwise_or(yellow, white)

        # Clean noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Row scan with filtering
        xs, ys = [], []
        prev_x = None

        misscount = 0

        for y in range(int(h * 0.9), int(h * 0.4), -5): # scan upwards every 5 pixels
            pts = np.where(mask[y] > 0)[0] 

            if len(pts):
                cx = int(np.mean(pts))
                misscount = 0

                # --- Outlier rejection ---
                if prev_x is None or abs(cx - prev_x) < 40:
                    xs.append(cx)
                    ys.append(y)
                    prev_x = cx
            else:
                misscount += 1
                if misscount > 10:  # if we've missed several rows in a row, stop
                    break

        # Fit curve
        curve = None
        out = image.copy()

        if len(xs) > 5:
            new_curve = np.polyfit(ys, xs, 2)

            if hasattr(self, "prev_curve") and self.prev_curve is not None:

                # Reject extreme jumps
                if abs(new_curve[0] - self.prev_curve[0]) > 0.002:
                    new_curve = self.prev_curve

                # Smooth
                new_curve = 0.7 * self.prev_curve + 0.3 * new_curve

            self.prev_curve = new_curve
            curve = new_curve

            # Draw curve for visualization
            for y in range(int(h * 0.9), int(h * 0.4), -1):
                x = int(np.polyval(curve, y))
                if 0 <= x < w:
                    cv2.circle(out, (x, y), 3, (0, 255, 0), -1)

        return curve, out
    
    def roi_from_black(self, frame):
        '''Alternative ROI method based on black region detection.'''
    
        h, w = frame.shape[:2]

        # Convert to HSV (better for black detection)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Black = low value (dark)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([220, 255, 200])  # tweak V=80 if needed

        black_mask = cv2.inRange(hsv, lower_black, upper_black)

        # Clean noise
        kernel = np.ones((7,7), np.uint8)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)

        # Find largest black region
        contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        roi_mask = np.zeros_like(black_mask)
        
        '''if contours:
            largest = max(contours, key=cv2.contourArea)

            # Optional: ignore tiny detections
            if cv2.contourArea(largest) > 500:
                cv2.drawContours(roi_mask, [largest], -1, 255, -1)'''
        
        num_labels, labels = cv2.connectedComponents(black_mask)

        roi_mask = np.zeros_like(black_mask)

        # Look at bottom row
        bottom_labels = labels[h-1]

        # Get unique labels touching bottom
        valid_labels = np.unique(bottom_labels)

        for label in valid_labels:
            if label == 0:  # skip background
                continue
            roi_mask[labels == label] = 255
            
        # Apply mask to original image
        roi = cv2.bitwise_and(frame, frame, mask=roi_mask)

        return roi, roi_mask
    
    def select_target_point(self, curve, image):

        if curve is None:
            return None

        h, w = image.shape[:2]

        # Lookahead point (tweak this for how far ahead you want to look)
        target_y = int(h * 0.5) # 0.5 means look halfway down the image
        target_x = int(np.polyval(curve, target_y))

        # Clamp
        target_x = max(0, min(target_x, w - 1))

        return (target_x, target_y)

    def draw_target_on_curve( self, curve_output_img, target_point):
        """
        Draw the target point on the curve output image.

        curve_output_img: np.array, BGR image with curve already drawn
        target_point: tuple (x, y), float or int
        """
        if target_point is not None:
            x, y = int(target_point[0]), int(target_point[1])
            # Draw a filled circle
            cv2.circle(curve_output_img, (x, y), 6, (255, 0, 0), -1)  # blue circle
            # Optional: draw crosshair
            cv2.line(curve_output_img, (x - 5, y), (x + 5, y), (255, 0, 0), 1)
            cv2.line(curve_output_img, (x, y - 5), (x, y + 5), (255, 0, 0), 1)

        return curve_output_img

    def birds_eye_view(self, frame):
        h, w = frame.shape[:2]

        # Define source points (trapezoid)
        src = np.float32([
            [int(0.4 * w), int(0.4 * h)],  # top left
            [int(0.6 * w), int(0.4 * h)],  # top right
            [int(1 * w), int(0.9 * h)],    # bottom right
            [int(0 * w), int(0.9 * h)]     # bottom left
        ])

        # Define destination points (rectangle)
        dst = np.float32([
            [int(0.3 * w), 0],              # top left
            [int(0.7 * w), 0],              # top right
            [int(0.7 * w), h],              # bottom right
            [int(0.3 * w), h]               # bottom left
        ])

        # Compute perspective transform matrix
        M = cv2.getPerspectiveTransform(src, dst)

        # Warp image to bird's eye view
        warped = cv2.warpPerspective(frame, M, (w, h))

        return warped

class LineDetectionNode(Node):

    def __init__(self):
        super().__init__('line_detection_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.image_publisher = self.create_publisher(
            Image,
            '/camera/line_image',
            10
        )

        self.point_publisher = self.create_publisher(
            geometry_msgs.msg.Point,
            '/vision/target_point',
            10
        )

        self.get_logger().info("Line detection node started.")

    def image_callback(self, msg):

        # Convert ROS image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        detector = LineDetector(frame)
        roi_frame = detector.apply_roi(frame) #original ROI method
        #roi_frame, roi_mask = detector.roi_from_black(roi_frame) #alternative ROI method

        # Run detection
        edges = detector.preprocess()
        lines = detector.detect_lines(edges)
        output = detector.draw_lines(lines) #also draws lines on output

        curve, curve_output = detector.detect_curve(roi_frame) #also draws curve points on output
        
        # Publish target point for pure pursuit
        target_point = detector.select_target_point(curve, frame)
        if target_point is not None:
            point_msg = geometry_msgs.msg.Point()
            point_msg.x = float(target_point[0])
            point_msg.y = float(target_point[1])
            point_msg.z = 0.0
            self.point_publisher.publish(point_msg)   

        # Draw target point on curve output for visualization
        curve_output = detector.draw_target_on_curve(curve_output, target_point)

        # Convert back to ROS message
        out_msg = self.bridge.cv2_to_imgmsg(curve_output, encoding='bgr8')
        self.image_publisher.publish(out_msg)
        
        


def main(args=None):
    rclpy.init(args=args)
    node = LineDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
