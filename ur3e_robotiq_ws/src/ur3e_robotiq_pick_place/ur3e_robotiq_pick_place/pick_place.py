import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import numpy as np
import time

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    RobotState,
)
from sensor_msgs.msg import JointState


class PickPlace(Node):
    def __init__(self):
        super().__init__("pick_place_demo")

        # MoveGroup action client (MoveIt2)
        self._move_group_client = ActionClient(
            self, MoveGroup, "move_group"
        )

        # Gripper command topic (matches your ros2_control yaml)
        self.gripper_pub = self.create_publisher(
            Float64, "/robotiq_gripper/command", 10
        )

        # Wait for action server
        self.get_logger().info("Waiting for move_group action server...")
        self._move_group_client.wait_for_server()
        self.get_logger().info("Connected to move_group.")

        time.sleep(1.0)

    # ------------------------------------------------------------------
    # Gripper helpers
    # ------------------------------------------------------------------
    def open_gripper(self):
        msg = Float64()
        msg.data = 0.0          # adjust for fully open
        self.get_logger().info("Opening gripper")
        self.gripper_pub.publish(msg)
        time.sleep(0.5)

    def close_gripper(self):
        msg = Float64()
        msg.data = 0.7          # adjust for fully closed
        self.get_logger().info("Closing gripper")
        self.gripper_pub.publish(msg)
        time.sleep(0.5)

    # ------------------------------------------------------------------
    # MoveIt helpers
    # ------------------------------------------------------------------
    def _send_joint_goal(self, group_name, joint_names, joint_values):
        """
        Low-level helper: send a joint goal to MoveIt using MoveGroup action.
        """
        # RobotState with joint positions
        start_state = RobotState()
        start_state.is_diff = False
        start_state.joint_state = JointState()
        start_state.joint_state.name = joint_names
        start_state.joint_state.position = joint_values

        # Constraints for goal state
        goal_constraints = Constraints()
        for name, val in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = 1e-3
            jc.tolerance_below = 1e-3
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        # MotionPlanRequest
        req = MotionPlanRequest()
        req.group_name = group_name
        req.start_state = start_state
        req.goal_constraints.append(goal_constraints)
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0

        # Build MoveGroup goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = True

        self.get_logger().info("Sending joint goal to MoveIt...")
        send_future = self._move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal was rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val != result.error_code.SUCCESS:
            self.get_logger().error(
                f"MoveGroup failed with error code: {result.error_code.val}"
            )
            return False

        self.get_logger().info("Motion executed successfully.")
        return True

    def move_to_joint_home(self):
        """
        Example: go to a simple 'home' configuration.
        Adjust the joint names if your MoveIt config uses a prefix.
        """
        # If your joints are prefixed (e.g. 'ur3e_shoulder_pan_joint'),
        # put those names here in the correct order that MoveIt expects.
        joint_names = [
            "ur3e_shoulder_pan_joint",
            "ur3e_shoulder_lift_joint",
            "ur3e_elbow_joint",
            "ur3e_wrist_1_joint",
            "ur3e_wrist_2_joint",
            "ur3e_wrist_3_joint",
        ]

        # radians; tune for a safe pose in front of the robot
        joint_values = [
            0.0,
            -1.57,
            1.57,
            0.0,
            0.0,
            0.0,
        ]

        return self._send_joint_goal("ur_manipulator", joint_names, joint_values)

    # ------------------------------------------------------------------
    # High-level demo sequence
    # ------------------------------------------------------------------
    def run_demo(self):
        self.get_logger().info("Starting pick & place demo sequence...")

        # 1) open gripper
        self.open_gripper()

        # 2) go to home joint pose
        if not self.move_to_joint_home():
            self.get_logger().error("Failed to move to home, aborting.")
            return

        # TODO: later we add:
        #   - move above object
        #   - move down
        #   - close gripper
        #   - move up
        #   - move to place pose
        #   - open gripper

        self.get_logger().info("Demo finished.")


def main(args=None):
    rclpy.init(args=args)
    node = PickPlace()
    try:
        node.run_demo()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
