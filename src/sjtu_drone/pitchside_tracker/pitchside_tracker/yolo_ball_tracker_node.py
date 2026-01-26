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


class YoloBallTracker(Node):
    def __init__(self):
        super().__init__('yolo_ball_tracker')
        self.ready_pub = self.create_publisher(Bool, '/yolo_tracker/ready', 1)
        self.bridge = CvBridge()
        self.model = YOLO("yolov8s.pt")
        self.ready_msg = Bool()
        self.ready_msg.data = False

        self.sub = self.create_subscription(Image, '/simple_drone/front/image_raw', self.image_cb, 10)
        self.pub = self.create_publisher(Point, 'ball_position', 10)

        self.last_log = time.time()
        self.get_logger().info("YOLO tracker started (bbox area enabled)")


    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        results = self.model(
            frame,
            device=0 if torch.cuda.is_available() else "cpu",
            imgsz=640,
            conf=0.18,
            verbose=False
        )
        # Publish READY once model is loaded
        if not self.ready_msg.data:
            self.ready_msg.data = True
            self.ready_pub.publish(self.ready_msg)
            self.get_logger().info("YOLO tracker READY")

        for r in results:
            if r.boxes is None:
                continue

            for box in r.boxes:
                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]
                if label != 'sports ball':
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                area = (x2 - x1) * (y2 - y1)
                print("ball area", area)

                p = Point()
                p.x = float(cx)
                p.y = float(cy)
                p.z = float(area)
                self.pub.publish(p)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.circle(frame, (cx, cy), 4, (0,0,255), -1)

        if time.time() - self.last_log > 2:
            self.get_logger().info("Ball detected and published")
            self.last_log = time.time()

        cv2.imshow("YOLO Ball Tracking", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloBallTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
