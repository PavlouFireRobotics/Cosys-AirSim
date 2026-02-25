#!/usr/bin/env python3
import time
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np

import airsim

from bt_airsim_interfaces.action import Takeoff, MoveDirection, FollowPath, TurnCamera


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

        self.move_as1 = ActionServer(
            self, MoveDirection, "move_direction",
            execute_callback=self.execute_move_direction,
            goal_callback=self._goal_ok,
            cancel_callback=self._cancel_ok,
            callback_group=self._cb_group
        )

        self.move_as2 = ActionServer(
            self, TurnCamera, "turn_camera",
            execute_callback=self.execute_turn_camera,
            goal_callback=self._goal_ok,
            cancel_callback=self._cancel_ok,
            callback_group=self._cb_group
        )

        self.follow_path_as = ActionServer(
            self, FollowPath, "follow_path",
            execute_callback=self.execute_follow_path,
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
            self.client.hoverAsync(vehicle_name=self.vehicle_name)
        except Exception:
            pass

    # ---------- actions ----------
    def execute_takeoff(self, goal_handle):
        feedback = Takeoff.Feedback()
        result = Takeoff.Result()

        def publish(status: str):
            feedback.status = status
            goal_handle.publish_feedback(feedback)

        with self._lock:
            try:
                publish("takeoff_start")

                # Kick off takeoff WITHOUT join, then poll state
                self.client.takeoffAsync(vehicle_name=self.vehicle_name).join()
                self._hover()
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

        # Absolute target position (AirSim world NED, meters)
        x_t = float(goal.twist.linear.x)
        y_t = float(goal.twist.linear.y)
        z_t = -3


        yaw = float(goal.twist.angular.z)  # deg


        self.get_logger().info(f"MoveDirection target (x,y): {x_t}, {y_t}, yaw_rate: {yaw}")

        feedback = MoveDirection.Feedback()
        result = MoveDirection.Result()

        def publish(status: str):
            feedback.status = status
            goal_handle.publish_feedback(feedback)

        with self._lock:
            try:
                publish("move_start")
                yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=yaw *180 / math.pi)
                self.client.moveToPositionAsync(
                    x_t, y_t, z_t,
                    velocity=0.5,
                    yaw_mode=yaw_mode,
                    vehicle_name=self.vehicle_name
                ).join()
                publish("move_complete")
                self._hover()
                publish("hovering")
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


    def execute_follow_path(self, goal_handle):
        goal = goal_handle.request

        feedback = FollowPath.Feedback()
        result = FollowPath.Result()

        def publish(status: str, idx: int = -1):
            feedback.status = status
            feedback.current_index = idx
            goal_handle.publish_feedback(feedback)

        # Build AirSim path
        path = []
        for i, p in enumerate(goal.points):
            path.append(airsim.Vector3r(float(p.x), float(p.y), -3.0))

        if len(path) < 2:
            goal_handle.abort()
            result.success = False
            result.message = "need at least 2 points"
            return result

        velocity = float(goal.velocity) if goal.velocity > 0.0 else 0.5
        timeout = float(goal.timeout_sec) if goal.timeout_sec > 0.0 else 1e9

        yaw_mode = airsim.YawMode(
            is_rate=bool(goal.yaw_is_rate),
            yaw_or_rate=float(goal.yaw_or_rate)
        )

        lookahead = float(goal.lookahead) if goal.lookahead > 0.0 else 1.0
        adaptive = bool(goal.adaptive_lookahead)

        with self._lock:
            try:
                publish("follow_path_start", 0)

                # Note: AirSim doesn't give per-waypoint callbacks here, so "current_index"
                # feedback can only be approximate unless you poll position and compute
                # nearest waypoint (can add that if you want).
                self.client.moveOnPathAsync(
                    path,
                    velocity=velocity,
                    timeout_sec=timeout,
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=yaw_mode,
                    lookahead=lookahead,
                    adaptive_lookahead=adaptive,
                    vehicle_name=self.vehicle_name
                ).join()

                publish("follow_path_complete", len(path) - 1)
                self._hover()

                goal_handle.succeed()
                result.success = True
                result.message = "follow_path complete"
                return result

            except Exception as e:
                self._hover()
                goal_handle.abort()
                result.success = False
                result.message = f"follow_path failed: {e}"
                return result


    def _wrap_to_pi(self, angle_rad: float) -> float:
        """Wrap angle to [-pi, pi]."""
        return (angle_rad + math.pi) % (2 * math.pi) - math.pi

    def execute_turn_camera(self, goal_handle):
        goal = goal_handle.request

        feedback = TurnCamera.Feedback()
        result = TurnCamera.Result()

        def publish(status: str):
            feedback.status = status
            goal_handle.publish_feedback(feedback)

        with self._lock:
            try:
                publish("turn_camera_start")

                st = self.client.getMultirotorState(vehicle_name=self.vehicle_name)
                current_yaw = airsim.to_eularian_angles(st.kinematics_estimated.orientation)[2]  # rad
                target_yaw = float(goal.target_yaw)  # rad

                # Wrap both (optional but helps consistency)
                current_yaw = self._wrap_to_pi(current_yaw)
                target_yaw  = self._wrap_to_pi(target_yaw)

                # Shortest signed angular difference target-current in [-pi, pi]
                delta = self._wrap_to_pi(target_yaw - current_yaw)

                rate = goal.rate if goal.rate > 0.0 else math.radians(self.max_yaw)  # rad/s

                self.get_logger().info(
                    f"Turning camera from {math.degrees(current_yaw):.1f} deg "
                    f"to {math.degrees(target_yaw):.1f} deg "
                    f"(delta {math.degrees(delta):.1f} deg) at {math.degrees(rate):.1f} deg/s"
                )

                duration = abs(delta) / rate
                yaw_rate_deg_s = math.copysign(math.degrees(rate), delta)

                # If already basically there, skip
                if abs(delta) < math.radians(0.5):  # 0.5 deg tolerance
                    goal_handle.succeed()
                    result.success = True
                    result.message = "turn_camera already at target"
                    return result

                self.client.rotateByYawRateAsync(
                    yaw_rate=yaw_rate_deg_s,   # deg/s
                    duration=duration,         # s
                    vehicle_name=self.vehicle_name
                ).join()

                goal_handle.succeed()
                result.success = True
                result.message = "turn_camera complete"
                return result

            except Exception as e:
                goal_handle.abort()
                result.success = False
                result.message = f"turn_camera failed: {e}"
                return result

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
