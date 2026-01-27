#!/usr/bin/env python3
import time
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import airsim

from bt_airsim_interfaces.action import Takeoff, MoveDirection


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class AirSimSimpleActions(Node):
    """
    Actions:
      - /takeoff        (airsim_msgs/Takeoff)
      - /move_direction (airsim_msgs/MoveDirection)

    move_direction:
      - BODY-frame velocity in m/s via geometry_msgs/Vector3 velocity
      - duration_sec
      - yaw_rate_deg_s (optional)
      - ROS +up is converted to AirSim NED (positive down)
    """

    def __init__(self):
        super().__init__("airsim_simple_actions")

        # --- Params ---
        self.declare_parameter("host_ip", "localhost")
        self.declare_parameter("vehicle_name", "Drone1")

        self.declare_parameter("max_xy_speed", 5.0)        # m/s
        self.declare_parameter("max_z_speed", 2.0)         # m/s (ROS +up)
        self.declare_parameter("max_yaw_rate_deg_s", 90.0) # deg/s

        self.host_ip = self.get_parameter("host_ip").value
        self.vehicle_name = self.get_parameter("vehicle_name").value
        self.max_xy = float(self.get_parameter("max_xy_speed").value)
        self.max_z = float(self.get_parameter("max_z_speed").value)
        self.max_yaw = float(self.get_parameter("max_yaw_rate_deg_s").value)

        # Serialize AirSim calls so actions don't fight each other
        self._lock = threading.Lock()
        self._cb_group = ReentrantCallbackGroup()

        # --- AirSim client ---
        self.client = airsim.MultirotorClient(ip=self.host_ip)
        self.client.confirmConnection()
        self.get_logger().info(f'Connected to AirSim at "{self.host_ip}" vehicle="{self.vehicle_name}"')

        self.client.enableApiControl(True, vehicle_name=self.vehicle_name)
        self.client.armDisarm(True, vehicle_name=self.vehicle_name)

        # --- Action servers ---
        self.takeoff_as = ActionServer(
            self, Takeoff, "takeoff",
            execute_callback=self.execute_takeoff,
            goal_callback=self._goal_ok,
            cancel_callback=self._cancel_ok,
            callback_group=self._cb_group
        )

        self.move_as = ActionServer(
            self, MoveDirection, "move_direction",
            execute_callback=self.execute_move_direction,
            goal_callback=self._goal_ok,
            cancel_callback=self._cancel_ok,
            callback_group=self._cb_group
        )

        self.get_logger().info("Ready: /takeoff, /move_direction")

    # ---------- common callbacks ----------
    def _goal_ok(self, _goal_request):
        return GoalResponse.ACCEPT

    def _cancel_ok(self, _goal_handle):
        return CancelResponse.ACCEPT

    # ---------- helpers ----------
    def _hover(self):
        try:
            self.client.hoverAsync(vehicle_name=self.vehicle_name).join()
        except Exception:
            pass

    # ---------- actions ----------
    def execute_takeoff(self, goal_handle):
        goal = goal_handle.request
        timeout = float(goal.timeout_sec) if goal.timeout_sec > 0.0 else 15.0
        target_alt = float(goal.target_altitude_m)

        feedback = Takeoff.Feedback()
        result = Takeoff.Result()

        with self._lock:
            start = time.time()
            try:
                feedback.status = "takeoff_start"
                goal_handle.publish_feedback(feedback)

                self.client.takeoffAsync(vehicle_name=self.vehicle_name).join()

                if target_alt > 0.0:
                    feedback.status = f"climb_to_{target_alt:.2f}m"
                    goal_handle.publish_feedback(feedback)

                    # AirSim NED: z positive down, so altitude up => negative z
                    target_z = -abs(target_alt)
                    self.client.moveToZAsync(
                        z=target_z,
                        velocity=max(0.5, self.max_z),
                        vehicle_name=self.vehicle_name
                    ).join()

                # bounded wait / cancel support
                while time.time() - start < timeout:
                    if goal_handle.is_cancel_requested:
                        self._hover()
                        goal_handle.canceled()
                        result.success = False
                        result.message = "takeoff canceled"
                        return result
                    break

                goal_handle.succeed()
                result.success = True
                result.message = "takeoff complete"
                return result

            except Exception as e:
                self._hover()
                goal_handle.abort()
                result.success = False
                result.message = f"takeoff failed: {e}"
                return result

    def execute_move_direction(self, goal_handle):
        goal = goal_handle.request

        # goal.twist: geometry_msgs/Twist
        vx = clamp(float(goal.twist.linear.x), -self.max_xy, self.max_xy)
        vy = clamp(float(goal.twist.linear.y), -self.max_xy, self.max_xy)

        # ROS: +up. AirSim NED: +down => invert
        vz_ros_up = clamp(float(goal.twist.linear.z), -self.max_z, self.max_z)
        vz_ned = -vz_ros_up

        duration = max(0.01, float(goal.duration_sec))
        yaw_rate = clamp(float(goal.twist.angular.z), -self.max_yaw, self.max_yaw)
        timeout = float(goal.timeout_sec) if goal.timeout_sec > 0.0 else max(5.0, duration + 2.0)

        feedback = MoveDirection.Feedback()
        result = MoveDirection.Result()

        with self._lock:
            start = time.time()
            try:
                # Stream small chunks so cancel is responsive
                step = 0.1  # seconds
                elapsed = 0.0

                while elapsed < duration:
                    if goal_handle.is_cancel_requested:
                        self._hover()
                        goal_handle.canceled()
                        result.success = False
                        result.message = "move canceled"
                        return result

                    if time.time() - start > timeout:
                        self._hover()
                        goal_handle.abort()
                        result.success = False
                        result.message = "move timeout"
                        return result

                    remaining = duration - elapsed
                    cmd_dt = min(step, remaining)

                    feedback.time_elapsed = float(elapsed)
                    goal_handle.publish_feedback(feedback)

                    yaw_mode = airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate)

                    self.client.moveByVelocityBodyFrameAsync(
                        vx, vy, vz_ned,
                        duration=cmd_dt,
                        yaw_mode=yaw_mode,
                        vehicle_name=self.vehicle_name
                    ).join()

                    elapsed = time.time() - start

                self._hover()
                goal_handle.succeed()
                result.success = True
                result.message = "move complete"
                return result

            except Exception as e:
                self._hover()
                goal_handle.abort()
                result.success = False
                result.message = f"move failed: {e}"
                return result

    def destroy_node(self):
        try:
            self._hover()
            self.client.armDisarm(False, vehicle_name=self.vehicle_name)
            self.client.enableApiControl(False, vehicle_name=self.vehicle_name)
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = AirSimSimpleActions()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
