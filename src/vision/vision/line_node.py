import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
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
    
    def highlight_white(self, image):
        """
        Highlight white regions in the image.
        Returns:
            mask: binary mask of white areas
            output: image with white highlighted
        """

        # Convert to HSV (better for color filtering)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define white range
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 40, 255])

        # Create mask
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # Clean noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Create highlighted output
        output = image.copy()
        output[mask > 0] = [0, 255, 0]  # highlight white as green

        return mask, output
    
    def get_road_edges_from_white(self, mask):
        h, w = mask.shape
        left_pts = []
        right_pts = []
        center_pts = []

        prev_center = None  # for temporal jump filtering

        edge_threshold = 10
        min_road_width = 0
        max_road_width = 30

        for y in range(int(h * 0.9), int(h * 0.5), -5):

            xs = np.where(mask[y] > 0)[0]
            if len(xs) == 0:
                continue

            # Identify contiguous white segments
            segments = np.split(xs, np.where(np.diff(xs) != 1)[0] + 1)

            # remove tiny noisy segments
            #segments = [s for s in segments if len(s) > 20]
            #if len(segments) < 2:
            #   continue

            left_white = segments[0]
            right_white = segments[-1]

            margin = 2
            left_edge = min(left_white[-1] + margin, w - 1)
            right_edge = max(right_white[0] - margin, 0)

            # 🚫 1. ignore edge-touching detections
            #if left_edge <= edge_threshold or right_edge >= (w - 1 - edge_threshold):
            #    continue

            road_width = right_edge - left_edge

            # 🚫 2. reject unrealistic widths
            #if road_width < min_road_width or road_width > max_road_width:
             #   continue

            center_x = (left_edge + right_edge) // 2

            # 🚫 3. temporal jump filtering (stability)
            if prev_center is not None:
                if abs(center_x - prev_center) > 50:
                    continue

            prev_center = center_x

            left_pts.append((left_edge, y))
            right_pts.append((right_edge, y))
            center_pts.append((center_x, y))

        return left_pts, right_pts, center_pts
    
    def draw_road_boundaries(self, image, left_pts, right_pts, center_pts=None):
        """
        Draw detected left and right road boundaries on the image.
        
        Args:
            image (np.array): Original BGR image
            left_pts (list of (x, y)): Points along the left road edge
            right_pts (list of (x, y)): Points along the right road edge
            center_pts (list of (x, y)): Points along the center road edge

        Returns:
            np.array: Copy of the image with boundaries drawn
        """
        out = image.copy()

        # Draw left edge in red
        for x, y in left_pts:
            cv2.circle(out, (x, y), 3, (0, 0, 255), -1)  # BGR: Red

        # Draw right edge in blue
        for x, y in right_pts:
            cv2.circle(out, (x, y), 3, (255, 0, 0), -1)  # BGR: Blue
        # Optionally, draw center line in green
        if center_pts is not None:
            for x, y in center_pts:
                cv2.circle(out, (x, y), 3, (255, 0, 255), -1)  # draw center points in magenta for visibility (BGR: Magenta)

        # Optionally, connect points with lines for clearer boundary visualization
        if len(left_pts) > 1:
            for i in range(1, len(left_pts)):
                cv2.line(out, left_pts[i-1], left_pts[i], (0, 0, 255), 2)
        if len(right_pts) > 1:
            for i in range(1, len(right_pts)):
                cv2.line(out, right_pts[i-1], right_pts[i], (255, 0, 0), 2)
        if center_pts is not None and len(center_pts) > 1:
            for i in range(1, len(center_pts)):
                cv2.line(out, center_pts[i-1], center_pts[i], (255, 0, 255), 2) # connect center points with magenta line

        return out
    
def method_1(self, frame, header):
        detector = LineDetector(frame)

        roi_frame = detector.apply_roi(frame) #original ROI method
        roi_frame, roi_mask = detector.roi_from_black(roi_frame) #alternative ROI method


        # Run detection on ROI frame 
        #edges = detector.preprocess()
        #lines = detector.detect_lines(edges)
        #output = detector.draw_lines(lines) #also draws lines on output

        curve, curve_output = detector.detect_curve(roi_frame) #also draws curve points on output
        
        # Publish target point for pure pursuit
        target_point = detector.select_target_point(curve, frame)
        if target_point is not None:
            point_msg = geometry_msgs.msg.PointStamped()
            point_msg.header = header  # preserve original timestamp and frame_id
            point_msg.point.x = float(target_point[0])
            point_msg.point.y = float(target_point[1])
            point_msg.point.z = 0.0
            self.point_publisher.publish(point_msg)
        else:
            self.get_logger().debug("No valid target point detected, skipping point publish.")

        # Highlight white regions
        mask, white_highlight_image = detector.highlight_white(frame)
        left_pts, right_pts, center_pts = detector.get_road_edges_from_white(mask)

        # Draw road boundaries
        boundary_image = detector.draw_road_boundaries(
            white_highlight_image,
            left_pts,
            right_pts,
            center_pts
        )

        # Draw target point ON TOP of same image
        boundary_image = detector.draw_target_on_curve(
            boundary_image,
            target_point
        )

        # Publish combined result
        out_msg = self.bridge.cv2_to_imgmsg(boundary_image, encoding='bgr8')
        out_msg.header = header  # preserve original timestamp and frame_id
        self.image_publisher.publish(out_msg)  


def method_2(self, frame, header):
        detector = LineDetector(frame)

        roi_frame = detector.apply_roi(frame) #original ROI method
        #roi_frame, roi_mask = detector.roi_from_black(roi_frame) #alternative ROI method

        # Run detection on ROI frame 
        #edges = detector.preprocess()
        #lines = detector.detect_lines(edges)
        #output = detector.draw_lines(lines) #also draws lines on output

        curve, curve_output = detector.detect_curve(roi_frame) #also draws curve points on output
        
        # Publish target point for pure pursuit
        target_point = detector.select_target_point(curve, frame)
        if target_point is not None:
            point_msg = geometry_msgs.msg.PointStamped()
            point_msg.header = header  # preserve original timestamp and frame_id
            point_msg.point.x = float(target_point[0])
            point_msg.point.y = float(target_point[1])
            point_msg.point.z = 0.0
            t_capture = rclpy.time.Time.from_msg(header.stamp)
            t_now = self.get_clock().now()
            delay = (t_now - t_capture).nanoseconds * 1e-9
            self.get_logger().info(f"Target point message latency: {delay:.3f} seconds")
            
            self.point_publisher.publish(point_msg)
        else:
            self.get_logger().debug("No valid target point detected, skipping point publish.")

        # Draw target point 
        curve_output = detector.draw_target_on_curve(
            curve_output,
            target_point
        )

        # Publish combined result
        out_msg = self.bridge.cv2_to_imgmsg(curve_output, encoding='bgr8')
        out_msg.header = header  # preserve original timestamp and frame_id
        self.image_publisher.publish(out_msg)  

def method_3(self, frame, header):
        # This method can be used to test a different ROI approach, such as using color-based segmentation to isolate the lane area instead of a fixed trapezoidal mask. You can implement it similarly to method_1 but with a different ROI logic, and then switch between them for testing.
        pass


class LineDetectionNode(Node):

    def __init__(self):
        super().__init__('line_detection_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            10
        )

        self.image_publisher = self.create_publisher(
            Image,
            '/camera/line_image',
            10
        )

        self.point_publisher = self.create_publisher(
            geometry_msgs.msg.PointStamped,
            '/vision/target_point',
            10
        )

        self.get_logger().info("Line detection node started.")

    def image_callback(self, msg):

        # Convert ROS image → OpenCV
        self.get_logger().debug("Received image frame, converting to OpenCV format.")
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)        
        self.get_logger().debug("Received image frame for processing.")

        #method_2(self, frame, msg.header)
        method_1(self, frame, msg.header)
        


def main(args=None):
    rclpy.init(args=args)
    node = LineDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
