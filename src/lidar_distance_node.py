#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32

def wrap_to_pi(a):
    """Wrap angle to [-pi, pi)."""
    while a >= math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def wrap_to_range(a, amin, amax):
    """Wrap angle 'a' into [amin, amax] given a 2π span."""
    two_pi = 2.0 * math.pi
    width = amax - amin  # should be ~2π
    while a < amin:
        a += two_pi
    while a > amax:
        a -= two_pi
    return a

class LidarDistanceNode(Node):
    """
    Subscribes:
      - /scan (LaserScan)
      - /target_angle_stamped (Vector3Stamped: z = yaw error in radians, frame camera_rgb_optical_frame)
    Publishes:
      - /target_range (Float32): meters
      - /target_polar (Vector3Stamped): x=range (m), y=theta (rad), z=0, header from scan
    """

    def __init__(self):
        super().__init__('lidar_distance_node')

        # Parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('angle_topic', '/target_angle_stamped')
        self.declare_parameter('range_topic', '/target_range')
        self.declare_parameter('polar_topic', '/target_polar')
        self.declare_parameter('window_deg', 5.0)        # +/- half-width around target angle
        self.declare_parameter('min_valid', 0.12)        # m (TB3 min range)
        self.declare_parameter('max_valid', 3.5)         # m (TB3 LDS useful range)
        self.declare_parameter('use_median', True)       # robust against outliers

        self.scan_topic   = self.get_parameter('scan_topic').value
        self.angle_topic  = self.get_parameter('angle_topic').value
        self.range_topic  = self.get_parameter('range_topic').value
        self.polar_topic  = self.get_parameter('polar_topic').value
        self.window_rad   = math.radians(float(self.get_parameter('window_deg').value))
        self.min_valid    = float(self.get_parameter('min_valid').value)
        self.max_valid    = float(self.get_parameter('max_valid').value)
        self.use_median   = bool(self.get_parameter('use_median').value)

        self.last_theta = None          # radians, in robot forward yaw sense
        self.last_scan  = None          # sensor_msgs/LaserScan

        # I/O
        self.sub_angle = self.create_subscription(
            Vector3Stamped, self.angle_topic, self.angle_cb, qos_profile_sensor_data)
        self.sub_scan = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)

        self.pub_range = self.create_publisher(Float32, self.range_topic, qos_profile_sensor_data)
        self.pub_polar = self.create_publisher(Vector3Stamped, self.polar_topic, qos_profile_sensor_data)

        self.get_logger().info(
            f'Listening: scan={self.scan_topic}, angle={self.angle_topic} | '
            f'Publishing: range={self.range_topic}, polar={self.polar_topic}')

    def angle_cb(self, msg: Vector3Stamped):
        self.last_theta = float(msg.vector.z)

        # If we already have a scan, try to compute distance immediately
        if self.last_scan is not None:
            self.compute_and_publish(self.last_scan, self.last_theta)

    def scan_cb(self, scan: LaserScan):
        self.last_scan = scan
        if self.last_theta is not None:
            self.compute_and_publish(scan, self.last_theta)

    def compute_and_publish(self, scan: LaserScan, theta: float):
        # Make sure theta is mapped into scan's angle range
        amin = scan.angle_min
        amax = scan.angle_max
        inc  = scan.angle_increment
        if inc == 0.0:
            return

        # Many scans are [-pi, +pi]; some are [0, 2pi). Be robust:
        theta_wrapped = wrap_to_range(theta, amin, amax)

        # Convert to index
        idx_center = int(round((theta_wrapped - amin) / inc))

        # Half window in indices
        half_win = max(1, int(round(self.window_rad / max(abs(inc), 1e-6))))

        # Collect ranges around target angle within window
        vals = []
        N = len(scan.ranges)
        for k in range(idx_center - half_win, idx_center + half_win + 1):
            i = max(0, min(N - 1, k))
            r = scan.ranges[i]
            if math.isfinite(r) and (self.min_valid <= r <= self.max_valid):
                vals.append(r)

        if not vals:
            # No valid returns in the window; skip publishing a bogus value
            return

        # Robust aggregation
        vals.sort()
        if self.use_median:
            m = vals[len(vals)//2]
        else:
            m = sum(vals) / len(vals)

        # Publish range
        out_r = Float32()
        out_r.data = float(m)
        self.pub_range.publish(out_r)

        # Publish polar (x=range, y=theta)
        out_p = Vector3Stamped()
        out_p.header = scan.header
        out_p.vector.x = float(m)
        out_p.vector.y = float(theta)  # original yaw error (not wrapped)
        out_p.vector.z = 0.0
        self.pub_polar.publish(out_p)


def main():
    rclpy.init()
    node = LidarDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
