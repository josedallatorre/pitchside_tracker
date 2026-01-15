import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO


class YoloBallTracker(Node):
    def __init__(self):
        super().__init__('yolo_ball_tracker')
        self.get_logger().info(f"torch.cuda.is_available(): {torch.cuda.is_available()}")
        self.get_logger().info(f"torch.version.cuda: {torch.version.cuda}")
        self.get_logger().info(f"torch.backends.cudnn.enabled: {torch.backends.cudnn.enabled}")

        if torch.cuda.is_available():
            self.get_logger().info(f"GPU: {torch.cuda.get_device_name(0)}")

        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('image_topic', '/simple_drone/front/image_raw')
        self.declare_parameter('model_path', 'yolov8s.pt')
        self.declare_parameter('confidence', 0.4)

        image_topic = self.get_parameter('image_topic').value
        model_path = self.get_parameter('model_path').value
        self.conf = self.get_parameter('confidence').value

        self.model = YOLO("yolov8s.pt")   # NOT nano
        self.device = "cuda" if torch.cuda.is_available() else "cpu"


        self.sub = self.create_subscription(
            Image, image_topic, self.image_cb, 10)

        self.pub = self.create_publisher(
            Point, 'ball_position', 10)

        self.get_logger().info('YOLO Ball Tracker started')

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        results = self.model(
        frame,
        device=0 if torch.cuda.is_available() else "cpu",
        imgsz=640,
        conf=0.18,
        iou=0.5,
        max_det=10,
        verbose=False
        )
        detected_labels = set()

        for r in results:
            if r.boxes is None:
                continue

            for box in r.boxes:
                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]
                detected_labels.add(label)

                if label != 'sports ball':
                    continue
                else:
                    print(label)

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # Publish ball center
                p = Point()
                p.x = float(cx)
                p.y = float(cy)
                p.z = 0.0
                self.pub.publish(p)

                # Visualization
                cv2.rectangle(frame, (x1, y1), (x2, y2),
                              (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                cv2.putText(frame, 'BALL',
                            (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)
        if detected_labels:
            self.get_logger().info(
                f"Detected labels: {sorted(detected_labels)}"
            )



        cv2.imshow('YOLO Ball Tracking', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloBallTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
