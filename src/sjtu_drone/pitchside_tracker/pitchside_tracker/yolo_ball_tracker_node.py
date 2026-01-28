import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO
from std_msgs.msg import Bool
import time
import threading


class YoloBallTracker(Node):
    def __init__(self):
        super().__init__('yolo_ball_tracker')

        self.bridge = CvBridge()
        self.model = YOLO("yolov8s.pt")

        self.ready_pub = self.create_publisher(Bool, '/yolo_tracker/ready', 1)
        self.pub = self.create_publisher(Point, 'ball_position', 10)

        self.sub = self.create_subscription(
            Image,
            '/simple_drone/front/image_raw',
            self.image_cb,
            10
        )

        self.ready_msg = Bool()
        self.ready_msg.data = False

        self.ball_track_id = None
        self.last_seen_time = 0.0
        self.track_timeout = 2.0

        # Shared frame (protected)
        self.frame_lock = threading.Lock()
        self.last_frame = None

        # GUI timer (30 FPS)
        self.create_timer(0.03, self.visualize)

        self.get_logger().info("YOLO Ball Tracker started")

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        results = self.model.track(
            frame,
            persist=True,
            device=0 if torch.cuda.is_available() else "cpu",
            imgsz=640,
            conf=0.18,
            tracker="bytetrack.yaml",
            verbose=False
        )

        if not self.ready_msg.data:
            self.ready_msg.data = True
            self.ready_pub.publish(self.ready_msg)
            self.get_logger().info("YOLO tracker READY")

        now = time.time()

        for r in results:
            if r.boxes is None or r.boxes.id is None:
                continue

            for box, track_id in zip(r.boxes, r.boxes.id):
                if self.model.names[int(box.cls[0])] != 'sports ball':
                    continue

                tid = int(track_id)

                if self.ball_track_id is None:
                    self.ball_track_id = tid

                if tid != self.ball_track_id:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                area = (x2 - x1) * (y2 - y1)

                self.last_seen_time = now

                p = Point(x=float(cx), y=float(cy), z=float(area))
                self.pub.publish(p)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                cv2.putText(
                    frame,
                    f"Ball ID {tid}",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 0, 0),
                    2
                )

        if self.ball_track_id and (now - self.last_seen_time > self.track_timeout):
            self.ball_track_id = None

        # Store frame safely
        with self.frame_lock:
            self.last_frame = frame.copy()

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
