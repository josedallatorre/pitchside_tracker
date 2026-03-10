import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
from ultralytics import YOLO
from std_msgs.msg import Bool
import time
import threading
from collections import deque


# ─── Tuning constants ────────────────────────────────────────────────────────
CONF_THRESHOLD   = 0.18      # low enough to catch weak detections
BALL_AREA_MIN    = 50        # px²
BALL_AREA_MAX    = 20000    # px² — raised from 8k; areas were hitting the ceiling
TRACK_TIMEOUT    = 3.0       # seconds before resetting the spatial prior
SMOOTH_ALPHA     = 0.40      # EMA weight for new detections
IMGSZ            = 1280      # small objects need high resolution
FRAME_SKIP       = 2         # process every Nth frame
MAX_JUMP_PX      = 300       # max distance (px) a detection can be from last known pos


class YoloBallTracker(Node):
    def __init__(self):
        super().__init__('yolo_ball_tracker')
        self.bridge = CvBridge()

        self.model = YOLO("yolov8s.pt")
        self.device = 0 if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Running on device: {self.device}")

        # Publishers / subscribers
        self.ready_pub = self.create_publisher(Bool, '/yolo_tracker/ready', 1)
        self.pub       = self.create_publisher(Point, 'ball_position', 10)
        self.sub       = self.create_subscription(
            Image, '/simple_drone/front/image_raw', self.image_cb, 10
        )

        # State
        self.ready_published = False
        self.last_seen_time  = 0.0
        self.frame_count     = 0

        # Smoothed position (EMA) — also serves as spatial prior for association
        self.smooth_cx: float | None = None
        self.smooth_cy: float | None = None
        self.smooth_area: float | None = None

        # Area history for adaptive thresholding (last 30 valid detections)
        self.area_history: deque[float] = deque(maxlen=30)

        # Shared frame for GUI
        self.frame_lock = threading.Lock()
        self.last_frame = None

        self.create_timer(0.033, self.visualize)   # ~30 FPS GUI
        cv2.namedWindow("YOLO Ball Tracking", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("YOLO Ball Tracking", 640, 360)
        self.get_logger().info("YOLO Ball Tracker started")

    # ── helpers ───────────────────────────────────────────────────────────────

    def _ema(self, prev: float | None, new: float) -> float:
        """Exponential moving average — smooths out jitter."""
        return new if prev is None else SMOOTH_ALPHA * new + (1 - SMOOTH_ALPHA) * prev

    def _area_ok(self, area: float) -> bool:
        """
        Validate detection area.
        Uses a 4x median tolerance to avoid rejecting valid detections
        when the drone moves toward/away from the ball.
        Only applies the median filter once we have >= 10 stable samples.
        """
        if not (BALL_AREA_MIN < area < BALL_AREA_MAX):
            return False
        if len(self.area_history) >= 10:
            median = float(np.median(self.area_history))
            if area > median * 4.0 or area < median / 4.0:
                return False
        return True

    def _dist_to_prior(self, box) -> float:
        """Euclidean distance from box centre to last smoothed position."""
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        return float(((cx - self.smooth_cx) ** 2 + (cy - self.smooth_cy) ** 2) ** 0.5)

    def _best_ball_box(self, results):
        """
        Return the best sports-ball detection using pure nearest-prior association.

        Strategy:
          - If we have a prior position: pick the closest valid detection within
            MAX_JUMP_PX. This rejects false positives that are far away.
          - If no prior (startup or after a long loss): pick the largest valid area.
        """
        candidates = []
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                if self.model.names[int(box.cls[0])] != 'sports ball':
                    continue
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                area = (x2 - x1) * (y2 - y1)
                if not self._area_ok(area):
                    continue
                candidates.append((box, area))

        if not candidates:
            return None

        if self.smooth_cx is not None:
            # Filter to detections within MAX_JUMP_PX of last known position
            near = [c for c in candidates if self._dist_to_prior(c[0]) < MAX_JUMP_PX]
            if near:
                # Among close candidates, pick the nearest one
                near.sort(key=lambda c: self._dist_to_prior(c[0]))
                return near[0][0]
            else:
                # Nothing plausible near last position — treat as lost
                return None

        # No prior yet — pick largest valid area
        candidates.sort(key=lambda c: c[1], reverse=True)
        return candidates[0][0]

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def image_cb(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % FRAME_SKIP != 0:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Pure detection — no tracker state, no ID management
        results = self.model.predict(
            frame,
            device=self.device,
            imgsz=IMGSZ,
            conf=CONF_THRESHOLD,
            verbose=False,
        )

        # Publish ready once
        if not self.ready_published:
            self.ready_published = True
            self.ready_pub.publish(Bool(data=True))
            self.get_logger().info("YOLO tracker READY")

        now = time.time()
        box = self._best_ball_box(results)

        if box is not None:
            self.last_seen_time = now

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            raw_cx   = float((x1 + x2) / 2)
            raw_cy   = float((y1 + y2) / 2)
            raw_area = float((x2 - x1) * (y2 - y1))

            # EMA smoothing
            self.smooth_cx   = self._ema(self.smooth_cx,   raw_cx)
            self.smooth_cy   = self._ema(self.smooth_cy,   raw_cy)
            self.smooth_area = self._ema(self.smooth_area, raw_area)
            self.area_history.append(raw_area)

            self.get_logger().info(
                f"[AREA DEBUG] raw={raw_area:.1f}  smooth={self.smooth_area:.1f}"
                f"  pos=({raw_cx:.0f},{raw_cy:.0f})"
            )

            self.pub.publish(Point(
                x=self.smooth_cx,
                y=self.smooth_cy,
                z=self.smooth_area,   # area ~ proxy for distance
            ))

            # Draw detection
            cx, cy = int(self.smooth_cx), int(self.smooth_cy)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(
                frame,
                f"area:{raw_area:.0f}  smooth:{self.smooth_area:.0f}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2,
            )
        else:
            lost_for = now - self.last_seen_time if self.last_seen_time else 0
            label = f"Ball LOST ({lost_for:.1f}s)"
            cv2.putText(
                frame, label, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2,
            )

        # Reset spatial prior after a long absence so startup logic kicks in
        # on next detection (picks largest area instead of nearest-to-stale-prior)
        if self.smooth_cx is not None and (now - self.last_seen_time > TRACK_TIMEOUT):
            self.get_logger().warn(
                f"Ball lost for >{TRACK_TIMEOUT}s — resetting spatial prior"
            )
            self.smooth_cx = self.smooth_cy = self.smooth_area = None
            self.area_history.clear()

        with self.frame_lock:
            self.last_frame = frame.copy()

    # ── GUI ───────────────────────────────────────────────────────────────────

    def visualize(self):
        with self.frame_lock:
            if self.last_frame is None:
                return
            frame = self.last_frame.copy()
        cv2.imshow("YOLO Ball Tracking", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloBallTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()