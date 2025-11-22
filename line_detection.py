import cv2
import numpy as np
import argparse
from typing import List, Tuple, Optional, Union

class LineDetector:
    """
    Simple line detector using Canny + Probabilistic Hough Transform.
    Accepts a file path or an image (numpy array).
    """

    def __init__(self, image: Union[str, np.ndarray], resize_width: Optional[int] = None):
        if isinstance(image, str):
            img = cv2.imread(image, cv2.IMREAD_COLOR)
            if img is None:
                raise FileNotFoundError(f"Could not read image: {image}")
        else:
            img = image.copy()
        if resize_width is not None and img.shape[1] != resize_width:
            h = int(img.shape[0] * (resize_width / img.shape[1]))
            img = cv2.resize(img, (resize_width, h), interpolation=cv2.INTER_AREA)
        self.orig = img
        if len(self.orig.shape) == 3 and self.orig.shape[2] == 3:
            self.gray = cv2.cvtColor(self.orig, cv2.COLOR_BGR2GRAY)
        else:
            self.gray = self.orig.copy()

    def preprocess(self, blur_ksize: int = 5, canny_thresh1: int = 50, canny_thresh2: int = 150) -> np.ndarray:
        """
        Apply Gaussian blur and Canny edge detector. Returns edge image.
        """
        if blur_ksize % 2 == 0:
            blur_ksize += 1
        blurred = cv2.GaussianBlur(self.gray, (max(1, blur_ksize), max(1, blur_ksize)), 0)
        edges = cv2.Canny(blurred, canny_thresh1, canny_thresh2)
        return edges

    def detect_lines(
        self,
        edges: Optional[np.ndarray] = None,
        rho: float = 1.0,
        theta: float = np.pi / 180,
        threshold: int = 50,
        min_line_length: int = 50,
        max_line_gap: int = 10
    ) -> List[Tuple[int, int, int, int]]:
        """
        Detect lines using HoughLinesP. Returns list of (x1, y1, x2, y2).
        """
        if edges is None:
            edges = self.preprocess()
        raw = cv2.HoughLinesP(edges, rho, theta, threshold, minLineLength=min_line_length, maxLineGap=max_line_gap)
        lines: List[Tuple[int, int, int, int]] = []
        if raw is None:
            return lines
        for l in raw:
            x1, y1, x2, y2 = int(l[0][0]), int(l[0][1]), int(l[0][2]), int(l[0][3])
            lines.append((x1, y1, x2, y2))
        return lines

    def draw_lines(self, lines: List[Tuple[int, int, int, int]], color: Tuple[int, int, int] = (0, 0, 255), thickness: int = 2) -> np.ndarray:
        """
        Returns a copy of the original image with lines drawn on it.
        """
        out = self.orig.copy()
        for x1, y1, x2, y2 in lines:
            cv2.line(out, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)
        return out

def nothing(x):
    pass

def create_trackbars(win: str):
    cv2.createTrackbar('blur', win, 5, 31, nothing)           # odd kernel size
    cv2.createTrackbar('canny1', win, 50, 500, nothing)
    cv2.createTrackbar('canny2', win, 150, 500, nothing)
    cv2.createTrackbar('rho', win, 1, 10, nothing)           # integer, converted to float
    cv2.createTrackbar('theta_deg', win, 1, 180, nothing)    # degrees
    cv2.createTrackbar('thresh', win, 50, 500, nothing)
    cv2.createTrackbar('min_len', win, 50, 1000, nothing)
    cv2.createTrackbar('max_gap', win, 10, 500, nothing)
    cv2.createTrackbar('thickness', win, 2, 10, nothing)

def get_trackbar_params(win: str):
    blur = cv2.getTrackbarPos('blur', win)
    if blur < 1:
        blur = 1
    if blur % 2 == 0:
        blur += 1
    canny1 = cv2.getTrackbarPos('canny1', win)
    canny2 = cv2.getTrackbarPos('canny2', win)
    rho = max(1, cv2.getTrackbarPos('rho', win))
    theta_deg = max(1, cv2.getTrackbarPos('theta_deg', win))
    theta = np.deg2rad(theta_deg)
    thresh = max(1, cv2.getTrackbarPos('thresh', win))
    min_len = max(1, cv2.getTrackbarPos('min_len', win))
    max_gap = max(0, cv2.getTrackbarPos('max_gap', win))
    thickness = max(1, cv2.getTrackbarPos('thickness', win))
    return {
        'blur': blur, 'canny1': canny1, 'canny2': canny2,
        'rho': float(rho), 'theta': float(theta), 'thresh': thresh,
        'min_len': min_len, 'max_gap': max_gap, 'thickness': thickness
    }

def process_and_show(det: LineDetector, win: str):
    params = get_trackbar_params(win)
    edges = det.preprocess(params['blur'], params['canny1'], params['canny2'])
    lines = det.detect_lines(edges, rho=params['rho'], theta=params['theta'], threshold=params['thresh'],
                             min_line_length=params['min_len'], max_line_gap=params['max_gap'])
    out = det.draw_lines(lines, (0, 0, 255), params['thickness'])
    cv2.imshow(win, out)
    cv2.imshow(win + ' - edges', edges)

def main():
    parser = argparse.ArgumentParser(description='Interactive line detection using OpenCV.')
    parser.add_argument('--image', type=str, help='Path to input image')
    parser.add_argument('--camera', type=int, help='Camera index (use for live webcam)')
    parser.add_argument('--resize', type=int, default=None, help='Resize width')
    args = parser.parse_args()

    if args.camera is not None:
        win = 'Line Detector (camera)'
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        create_trackbars(win)
        cap = cv2.VideoCapture(args.camera)
        if not cap.isOpened():
            raise RuntimeError('Could not open camera')
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            det = LineDetector(frame, resize_width=args.resize)
            process_and_show(det, win)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
        cap.release()
    elif args.image:
        img = cv2.imread(args.image, cv2.IMREAD_COLOR)
        if img is None:
            raise FileNotFoundError(f"Could not read image: {args.image}")
        win = 'Line Detector'
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        create_trackbars(win)
        det = LineDetector(img, resize_width=args.resize)
        while True:
            process_and_show(det, win)
            key = cv2.waitKey(50) & 0xFF
            if key == 27:  # ESC
                break
            if key == ord('s'):
                params = get_trackbar_params(win)
                edges = det.preprocess(params['blur'], params['canny1'], params['canny2'])
                lines = det.detect_lines(edges, rho=params['rho'], theta=params['theta'],
                                         threshold=params['thresh'], min_line_length=params['min_len'],
                                         max_line_gap=params['max_gap'])
                out = det.draw_lines(lines, (0, 0, 255), params['thickness'])
                save_path = 'lines_out.jpg'
                cv2.imwrite(save_path, out)
                print(f"Saved {save_path}")
    else:
        print("Provide --image <path> or --camera <index>")

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()