#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import math
from ultralytics import YOLO

class DistanceFromYOLO(Node):
    """
    Estimate distance to the detected person using bounding-box size.
    Publishes: /target_distance (Float32, in meters)
    """
    def __init__(self):
        super().__init__('distance_from_yolo')
        self.bridge = CvBridge()

        # --- Params
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('distance_topic', '/target_distance')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('H_real', 1.7)        # average human height (m)
        self.declare_parameter('focal_length', 600.0) # pixels
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('imgsz', 640)

        # --- Load
        image_topic = self.get_parameter('image_topic').value
        distance_topic = self.get_parameter('distance_topic').value
        model_path = self.get_parameter('model_path').value
        self.H_real = float(self.get_parameter('H_real').value)
        self.focal_length = float(self.get_parameter('focal_length').value)
        self.conf = float(self.get_parameter('conf').value)
        self.imgsz = int(self.get_parameter('imgsz').value)

        self.model = YOLO(model_path)
        self.get_logger().info(f"Loaded YOLOv8 model {model_path}")

        # --- Pub/Sub
        self.sub = self.create_subscription(Image, image_topic, self.image_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Float32, distance_topic, qos_profile_sensor_data)

        self.get_logger().info(f"Subscribed to {image_topic}, publishing {distance_topic}")

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        results = self.model(frame, imgsz=self.imgsz, conf=self.conf, classes=[0])  # class 0 = person

        if len(results) and len(results[0].boxes):
            boxes = results[0].boxes
            for i in range(len(boxes)):
                x1, y1, x2, y2 = [int(v) for v in boxes.xyxy[i].tolist()]
                conf = float(boxes.conf[i].item())

                H_pixels = y2 - y1
                if H_pixels <= 0:
                    continue

                # Distance estimation
                d = (self.H_real * self.focal_length) / H_pixels
                msg_out = Float32()
                msg_out.data = float(d)
                self.pub.publish(msg_out)

                self.get_logger().info(f"person: H_pix={H_pixels}, dist={d:.2f}m (conf={conf:.2f})")
                break  # only first person
        else:
            msg_out = Float32()
            msg_out.data = float('nan')
            self.pub.publish(msg_out)

def main():
    rclpy.init()
    node = DistanceFromYOLO()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
