#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO

import torch
from geometry_msgs.msg import PointStamped

class YoloPersonDetector(Node):
    def __init__(self):
        super().__init__('yolo_person_detector')
        self.bridge = CvBridge()

        # Params
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('annotated_topic', '/camera/image_yolo')  # <-- RViz “Image_yolo”
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('imgsz', 640)

        image_topic     = self.get_parameter('image_topic').get_parameter_value().string_value
        annotated_topic = self.get_parameter('annotated_topic').get_parameter_value().string_value
        model_path      = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf       = float(self.get_parameter('conf').value)
        self.imgsz      = int(self.get_parameter('imgsz').value)

        # Loading YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)

        #  Check if system is using CUDA or GPU 
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'YOLO running on device: {device}')

        # Sub/Pub with sensor-data QoS (Best Effort, low latency)
        self.sub = self.create_subscription(Image, image_topic, self.image_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Image, annotated_topic, qos_profile_sensor_data)
        
        # Publishing center pixel of bounding box (u, v)
        self.pub_center = self.create_publisher(PointStamped, '/target_pixel', qos_profile_sensor_data)

        self.get_logger().info(f'Subscribed: {image_topic}')
        self.get_logger().info(f'Annotated out: {annotated_topic}')

    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        annotated = frame.copy()

        # Inference: only "person" class (COCO id 0)
        results = self.model(annotated, imgsz=self.imgsz, conf=self.conf, classes=[0])

        if results and len(results):
            boxes = results[0].boxes
            if boxes is not None and boxes.xyxy is not None:
                for i in range(len(boxes)):
                    x1, y1, x2, y2 = [int(v) for v in boxes.xyxy[i].tolist()]
                    conf = float(boxes.conf[i].item()) if boxes.conf is not None else 0.0
                    u = (x1 + x2) // 2
                    v = (y1 + y2) // 2

                    # Publish center of bounding box
                    pt = PointStamped()
                    pt.header.stamp = msg.header.stamp
                    pt.header.frame_id = msg.header.frame_id
                    pt.point.x = float(u)  # pixel u
                    pt.point.y = float(v)  # pixel v
                    pt.point.z = 0.0
                    self.pub_center.publish(pt)

                    # Log center to terminal
                    self.get_logger().info(f'person: center=({u},{v}) conf={conf:.2f}')

                    # Draw overlay
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.circle(annotated, (u, v), 4, (0, 255, 0), -1)
                    cv2.putText(annotated, f'person {conf:.2f}', (x1, max(0, y1 - 7)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 230, 60), 2, cv2.LINE_AA)

        # Publish annotated image (even if no detections)
        out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out.header = Header()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        self.pub.publish(out)

def main():
    rclpy.init()
    node = YoloPersonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
