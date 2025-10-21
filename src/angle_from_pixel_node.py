#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped, Vector3Stamped
from std_msgs.msg import Float32

class AngleFromPixel(Node):
    def __init__(self):
        super().__init__('angle_from_pixel')

        # Params (override if your topics differ)
        self.declare_parameter('pixel_topic', '/target_pixel')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('angle_topic', '/target_angle')                  # plain float
        self.declare_parameter('angle_stamped_topic', '/target_angle_stamped')  # stamped vector (z=yaw)

        pixel_topic = self.get_parameter('pixel_topic').value
        caminfo_topic = self.get_parameter('camera_info_topic').value
        angle_topic = self.get_parameter('angle_topic').value
        angle_stamped_topic = self.get_parameter('angle_stamped_topic').value

        self.fx = None
        self.cx = None

        self.sub_caminfo = self.create_subscription(
            CameraInfo, caminfo_topic, self.caminfo_cb, qos_profile_sensor_data)
        self.sub_pixel = self.create_subscription(
            PointStamped, pixel_topic, self.pixel_cb, qos_profile_sensor_data)

        self.pub_angle = self.create_publisher(Float32, angle_topic, qos_profile_sensor_data)
        self.pub_angle_stamped = self.create_publisher(Vector3Stamped, angle_stamped_topic, qos_profile_sensor_data)

        self.get_logger().info(f'Listening: pixel={pixel_topic}, caminfo={caminfo_topic}')
        self.get_logger().info(f'Publishing: {angle_topic} (Float32), {angle_stamped_topic} (Vector3Stamped z=yaw)')

    def caminfo_cb(self, msg: CameraInfo):
        # K = [fx, 0, cx,  0, fy, cy,  0, 0, 1]
        self.fx = msg.k[0]
        self.cx = msg.k[2]

    def pixel_cb(self, msg: PointStamped):
        if self.fx is None or self.cx is None:
            # Wait until we have intrinsics
            return

        u = msg.point.x
        theta_err = math.atan((u - self.cx) / self.fx)  # radians, yaw error

        # Plain float
        out = Float32()
        out.data = float(theta_err)
        self.pub_angle.publish(out)

        # Stamped vector: put yaw in z
        out_s = Vector3Stamped()
        out_s.header = msg.header # keep original timestamp/frame
        out_s.vector.x = 0.0
        out_s.vector.y = 0.0
        out_s.vector.z = float(theta_err)
        self.pub_angle_stamped.publish(out_s)

def main():
    rclpy.init()
    node = AngleFromPixel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
