#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import airsim


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class AirSimTwistBridge(Node):
    """
    Subscribes to geometry_msgs/Twist (cmd_vel) and sends velocity+yaw-rate commands to AirSim.

    Mapping (typical ROS mobile base convention):
      - twist.linear.x  -> forward velocity (m/s)
      - twist.linear.y  -> left velocity (m/s)  (we treat as right if your controls feel flipped)
      - twist.linear.z  -> up velocity (m/s)    (optional, teleop_twist_keyboard can publish this)
      - twist.angular.z -> yaw rate (rad/s)

    AirSim expects velocities in m/s, duration in seconds.
    """
    def __init__(self):
        super().__init__('airsim_twist_bridge')

        # Parameters
        self.declare_parameter('vehicle_name', 'Drone1')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('command_rate_hz', 20.0)
        self.declare_parameter('max_xy_speed', 5.0)      # m/s
        self.declare_parameter('max_z_speed', 2.0)       # m/s (up/down)
        self.declare_parameter('max_yaw_rate', 1.5)      # rad/s
        self.declare_parameter('takeoff_on_start', True)
        self.declare_parameter('use_world_frame', True)  # if False, you can switch to body frame behavior later
        self.declare_parameter('host_ip', 'localhost') 

        self.vehicle_name = self.get_parameter('vehicle_name').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.rate_hz = float(self.get_parameter('command_rate_hz').value)
        self.max_xy = float(self.get_parameter('max_xy_speed').value)
        self.max_z = float(self.get_parameter('max_z_speed').value)
        self.max_yaw = float(self.get_parameter('max_yaw_rate').value)
        self.takeoff_on_start = bool(self.get_parameter('takeoff_on_start').value)
        self.use_world_frame = bool(self.get_parameter('use_world_frame').value)

        # State from last Twist
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0

        # Connect to AirSim
        self.client = airsim.MultirotorClient(ip=self.get_parameter('host_ip').get_parameter_value().string_value)
        self.client.confirmConnection()

        self.get_logger().info(f'Connected to AirSim. Using vehicle_name="{self.vehicle_name}"')

        # Enable API and arm
        self.client.enableApiControl(True, )
        self.client.armDisarm(True, )

        # Optional takeoff
        if self.takeoff_on_start:
            self.get_logger().info('Taking off...')
            self.client.takeoffAsync().join()

        # ROS interfaces
        self.sub = self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 10)

        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f'Subscribed to {self.cmd_vel_topic}. Sending commands at {self.rate_hz:.1f} Hz.'
        )

    def on_cmd_vel(self, msg: Twist):
        # Clamp speeds
        self.vx = clamp(msg.linear.x, -self.max_xy, self.max_xy)

        # NOTE: Many users want A/D to be left/right. teleop_twist_keyboard publishes linear.y for left/right.
        # If your lateral direction feels inverted, flip the sign here.
        self.vy = clamp(msg.linear.y, -self.max_xy, self.max_xy)

        # teleop_twist_keyboard can publish linear.z if you use i/k keys (depending on its mode)
        # We interpret +z as up in ROS, but AirSim's "z" in moveByVelocity is NED (positive down).
        # So we invert: up (ROS +z) => negative z velocity in AirSim.
        vz_up_ros = clamp(msg.linear.z, -self.max_z, self.max_z)
        self.vz = -vz_up_ros

        self.yaw_rate = clamp(msg.angular.z, -self.max_yaw, self.max_yaw)

    def on_timer(self):
        duration = 1.0 / self.rate_hz

        # Convert yaw rate rad/s -> deg/s for AirSim yaw_mode
        yaw_rate_deg = self.yaw_rate * (180.0 / math.pi)

        yaw_mode = airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate_deg)

        try:
            # World-frame velocity command
            # (For most keyboard teleop this feels “stable”. If you want body-frame controls, we can change strategy.)
            self.client.moveByVelocityBodyFrameAsync(
                self.vx, self.vy, self.vz,
                duration=duration,
                yaw_mode=yaw_mode,
                
            )
        except Exception as e:
            self.get_logger().error(f'AirSim command failed: {e}')

    def destroy_node(self):
        # Clean shutdown: stop and release control
        try:
            self.client.hoverAsync().join()
            self.client.armDisarm(False, )
            self.client.enableApiControl(False, )
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = AirSimTwistBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
