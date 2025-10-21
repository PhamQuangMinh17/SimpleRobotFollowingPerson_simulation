#!/usr/bin/env python3
import math
import math as _math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.duration import Duration

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class FollowPIDController(Node):
    def __init__(self):
        super().__init__('follow_pid_controller')

        # --------------------
        # Topics & core params
        # --------------------
        self.declare_parameter('angle_topic', '/target_angle')          # std_msgs/Float32 (rad)
        self.declare_parameter('distance_topic', '/target_distance')    # std_msgs/Float32 (m) (NaN if unknown)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # Yaw PID
        self.declare_parameter('yaw_kp', 1.2)
        self.declare_parameter('yaw_ki', 0.0)
        self.declare_parameter('yaw_kd', 0.15)
        self.declare_parameter('yaw_tol_rad', 0.03)        # stop rotating when |error| < tol
        self.declare_parameter('yaw_hold_time', 0.25)      # seconds inside tol to declare done
        self.declare_parameter('max_ang_speed', 1.2)       # rad/s clamp
        self.declare_parameter('reverse_cmd', True)        # << you said true works in your sim

        # Distance PID
        self.declare_parameter('desired_distance', 2.0)    # meters
        self.declare_parameter('dist_kp', 0.6)
        self.declare_parameter('dist_ki', 0.0)
        self.declare_parameter('dist_kd', 0.05)
        self.declare_parameter('dist_tol_m', 0.05)         # stop translating when within
        self.declare_parameter('max_lin_speed', 0.35)      # m/s clamp

        # Behaviour/safety
        self.declare_parameter('angle_gate_rad', 0.35)     # if |angle| > gate, hold linear = 0 and just rotate
        self.declare_parameter('angle_timeout_sec', 0.5)   # if angle stale, stop
        self.declare_parameter('dist_timeout_sec', 0.5)    # if distance stale, set v=0 (but can still yaw)
        self.declare_parameter('rate_hz', 50.0)

        # Read params
        self.angle_topic   = self.get_parameter('angle_topic').value
        self.distance_topic = self.get_parameter('distance_topic').value
        self.cmd_topic     = self.get_parameter('cmd_vel_topic').value

        self.yaw_kp   = float(self.get_parameter('yaw_kp').value)
        self.yaw_ki   = float(self.get_parameter('yaw_ki').value)
        self.yaw_kd   = float(self.get_parameter('yaw_kd').value)
        self.yaw_tol  = float(self.get_parameter('yaw_tol_rad').value)
        self.yaw_hold = float(self.get_parameter('yaw_hold_time').value)
        self.max_w    = float(self.get_parameter('max_ang_speed').value)
        self.reverse  = bool(self.get_parameter('reverse_cmd').value)

        self.d_des    = float(self.get_parameter('desired_distance').value)
        self.dist_kp  = float(self.get_parameter('dist_kp').value)
        self.dist_ki  = float(self.get_parameter('dist_ki').value)
        self.dist_kd  = float(self.get_parameter('dist_kd').value)
        self.dist_tol = float(self.get_parameter('dist_tol_m').value)
        self.max_v    = float(self.get_parameter('max_lin_speed').value)

        self.angle_gate = float(self.get_parameter('angle_gate_rad').value)
        self.angle_timeout = float(self.get_parameter('angle_timeout_sec').value)
        self.dist_timeout  = float(self.get_parameter('dist_timeout_sec').value)
        self.rate_hz       = float(self.get_parameter('rate_hz').value)

        # --------------------
        # Subscriptions
        # --------------------
        self.err_yaw = 0.0
        self.last_angle_time = None
        self.create_subscription(Float32, self.angle_topic, self._angle_cb, qos_profile_sensor_data)

        self.last_dist = float('nan')
        self.last_dist_time = None
        self.create_subscription(Float32, self.distance_topic, self._distance_cb, qos_profile_sensor_data)

        # --------------------
        # Publisher (RELIABLE for /cmd_vel)
        # --------------------
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pub = self.create_publisher(Twist, self.cmd_topic, cmd_qos)

        # --------------------
        # PID state
        # --------------------
        # Yaw PID state
        self.yaw_prev_err = 0.0
        self.yaw_int = 0.0
        self.yaw_in_tol_since = None
        self.yaw_done = False

        # Distance PID state
        self.dist_prev_err = 0.0
        self.dist_int = 0.0

        # Loop timer
        self.dt = 1.0 / self.rate_hz
        self.create_timer(self.dt, self._step)

        self.get_logger().info(
            f'Follow controller:\n'
            f'  angle={self.angle_topic}, distance={self.distance_topic} → {self.cmd_topic}\n'
            f'  yaw PID kp={self.yaw_kp}, ki={self.yaw_ki}, kd={self.yaw_kd}, tol={self.yaw_tol} rad, reverse={self.reverse}\n'
            f'  dist PID kp={self.dist_kp}, ki={self.dist_ki}, kd={self.dist_kd}, d*={self.d_des} m'
        )

    # --------------------
    # Callbacks
    # --------------------
    def _angle_cb(self, msg: Float32):
        self.err_yaw = float(msg.data)  # left<0, right>0
        self.last_angle_time = self.get_clock().now()

        # If we were holding and error grows, re-engage yaw loop
        if self.yaw_done and abs(self.err_yaw) > 1.5 * self.yaw_tol:
            self.yaw_done = False
            self.yaw_in_tol_since = None

    def _distance_cb(self, msg: Float32):
        self.last_dist = float(msg.data)  # meters (may be NaN)
        self.last_dist_time = self.get_clock().now()

    # 
    # Control step
    # 
    def _step(self):
        now = self.get_clock().now()
        twist = Twist()

        #  Yaw PID (always active if angle fresh) 
        yaw_cmd = 0.0
        angle_fresh = (self.last_angle_time is not None and
                       (now - self.last_angle_time).nanoseconds * 1e-9 <= self.angle_timeout)

        if angle_fresh:
            e = self.err_yaw
            # PID
            self.yaw_int = _clamp(self.yaw_int + e * self.dt, -2.0, 2.0)
            d_err = (e - self.yaw_prev_err) / self.dt
            pid = self.yaw_kp * e + self.yaw_ki * self.yaw_int + self.yaw_kd * d_err

            # Sign: you validated reverse=True works. Keep that behaviour.
            yaw_cmd = (-pid) if self.reverse else (pid)
            yaw_cmd = _clamp(yaw_cmd, -self.max_w, self.max_w)

            # Tolerance / hold
            if abs(e) <= self.yaw_tol:
                if self.yaw_in_tol_since is None:
                    self.yaw_in_tol_since = now
                hold = (now - self.yaw_in_tol_since).nanoseconds * 1e-9
                yaw_cmd = 0.0
                if hold >= self.yaw_hold:
                    self.yaw_done = True
            else:
                self.yaw_in_tol_since = None
                self.yaw_done = False

            self.yaw_prev_err = e
        else:
            # angle stale -> no rotation
            yaw_cmd = 0.0

        #  Distance PID (run only if distance fresh and finite) 
        lin_cmd = 0.0
        dist_fresh = (self.last_dist_time is not None and
                      (now - self.last_dist_time).nanoseconds * 1e-9 <= self.dist_timeout)
        dist_valid = dist_fresh and _isfinite(self.last_dist)

        if dist_valid:
            e_d = self.last_dist - self.d_des     # + => too far, drive forward
            # Gate linear speed if angle is large (prioritize turning)
            if angle_fresh and abs(self.err_yaw) > self.angle_gate:
                e_d_eff = 0.0   # lock linear when facing error is large
            else:
                e_d_eff = e_d

            # PID for distance
            self.dist_int = _clamp(self.dist_int + e_d_eff * self.dt, -1.5, 1.5)
            d_d = (e_d_eff - self.dist_prev_err) / self.dt
            lin_cmd = self.dist_kp * e_d_eff + self.dist_ki * self.dist_int + self.dist_kd * d_d

            # Deadband near target
            if abs(e_d) <= self.dist_tol:
                lin_cmd = 0.0

            lin_cmd = _clamp(lin_cmd, -self.max_v, self.max_v)
            self.dist_prev_err = e_d_eff
        else:
            lin_cmd = 0.0  # no distance -> don't translate

        # --------- Publish command ---------
        twist.angular.z = yaw_cmd
        twist.linear.x  = lin_cmd
        self.pub.publish(twist)


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def _isfinite(x):
    return not (math.isnan(x) or math.isinf(x))


def main():
    rclpy.init()
    node = FollowPIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
