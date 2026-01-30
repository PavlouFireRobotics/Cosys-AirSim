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

        self.move_as = ActionServer(
            self, MoveDirection, "turn_camera",
            execute_callback=self.execute_turn_camera,
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


        yaw_rate = float(goal.twist.angular.z)  # deg/s (rate)

        self.get_logger().info(f"MoveDirection target (x,y): {x_t}, {y_t}, yaw_rate: {yaw_rate}")

        feedback = MoveDirection.Feedback()
        result = MoveDirection.Result()

        def publish(status: str):
            feedback.status = status
            goal_handle.publish_feedback(feedback)

        with self._lock:
            try:
                publish("move_start")

                yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=yaw_rate)
                self.client.moveToPositionAsync(
                    x_t, y_t, z_t,
                    velocity=0.5,
                    yaw_mode=yaw_mode,
                    vehicle_name=self.vehicle_name
                ).join()

                self.client.rotateToYawAsync(
                    yaw_rate,
                    vehicle_name=self.vehicle_name
                ).join()

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


    def execute_turn_camera(self, goal_handle):
        goal = goal_handle.request

        feedback = MoveDirection.Feedback()
        result = MoveDirection.Result()

        def publish(status: str):
            feedback.status = status
            goal_handle.publish_feedback(feedback)

        with self._lock:
            try:
                publish("turn_camera_start")
                yaw = float(goal.twist.angular.z)    # degrees

                st = self.client.getMultirotorState(vehicle_name=self.vehicle_name)
                current_yaw = math.degrees(airsim.to_eularian_angles(st.kinematics_estimated.orientation)[2])
                target_yaw = current_yaw + yaw

                self.client.rotateToYawAsync(
                    target_yaw,
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
