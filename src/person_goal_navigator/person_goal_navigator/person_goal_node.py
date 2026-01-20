#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
import numpy as np
import time
import os


class PersonGoalNode(Node):

    def __init__(self):
        super().__init__('person_goal_node')

        self.get_logger().info("Person Goal Navigator started")

        # -------------------------
        # Parameters
        # -------------------------
        self.num_samples = 10

        self.LIN_VEL_TH = 0.02
        self.ANG_VEL_TH = 0.02

        # -------------------------
        # Robot motion state
        # -------------------------
        self.robot_stopped = False

        # -------------------------
        # Mission timing
        # -------------------------
        self.start_time = time.time()
        self.person_found_time = None
        self.goal_reached_time = None

        self.person_x = None
        self.person_y = None

        # -------------------------
        # Storage
        # -------------------------
        self.poses = []
        self.goal_sent = False

        # -------------------------
        # Subscribers
        # -------------------------
        self.sub_person = self.create_subscription(
            PoseStamped,
            '/person_side',
            self.person_callback,
            10
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            '/a200_0000/platform/odom/filtered',
            self.odom_callback,
            10
        )

        # -------------------------
        # Nav2 Action Client
        # -------------------------
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/a200_0000/navigate_to_pose'
        )

    # =====================================================
    # ODOM CALLBACK
    # =====================================================
    def odom_callback(self, msg: Odometry):

        v = abs(msg.twist.twist.linear.x)
        w = abs(msg.twist.twist.angular.z)

        self.robot_stopped = (
            v < self.LIN_VEL_TH and
            w < self.ANG_VEL_TH
        )

    # =====================================================
    # PERSON CALLBACK
    # =====================================================
    def person_callback(self, msg: PoseStamped):

        #PERSON FOUND (first message)
        if self.person_found_time is None:
            self.person_found_time = time.time()
            self.person_x = msg.pose.position.x
            self.person_y = msg.pose.position.y

            self.get_logger().info(
                f"Person FOUND at x={self.person_x:.2f}, y={self.person_y:.2f}"
            )

        #Only collect samples if robot is stopped
        if not self.robot_stopped:
            self.get_logger().debug("Robot moving — ignoring sample")
            return

        if self.goal_sent:
            return

        self.poses.append([
            msg.pose.position.x,
            msg.pose.position.y
        ])

        self.get_logger().info(
            f"Sample accepted {len(self.poses)}/{self.num_samples}"
        )

        if len(self.poses) >= self.num_samples:
            self.compute_and_send_goal(msg)

    # =====================================================
    # COMPUTE MEAN GOAL
    # =====================================================
    def compute_and_send_goal(self, last_msg: PoseStamped):

        poses_np = np.array(self.poses)

        mean_x = np.mean(poses_np[:, 0])
        mean_y = np.mean(poses_np[:, 1])

        self.get_logger().info(
            f"Averaged goal: x={mean_x:.2f}, y={mean_y:.2f}"
        )

        self.send_navigation_goal(
            mean_x,
            mean_y,
            last_msg.pose.orientation
        )

        self.goal_sent = True

    # =====================================================
    # SEND NAV GOAL
    # =====================================================
    def send_navigation_goal(self, x, y, orientation):

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation = orientation

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info("Sending navigation goal")

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    # =====================================================
    # GOAL RESPONSE
    # =====================================================
    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    # =====================================================
    # GOAL RESULT
    # =====================================================
    def goal_result_callback(self, future):

        self.goal_reached_time = time.time()

        status = future.result().status
        self.get_logger().info(f"Navigation finished with status: {status}")

        self.generate_report()

    # =====================================================
    # REPORT
    # =====================================================
    def generate_report(self):

        total_time = time.time() - self.start_time
        report_path = os.path.expanduser('~/mission_report.txt')

        with open(report_path, 'w') as f:
            f.write('Mission Report\n')
            f.write('==============\n\n')

            f.write('Person found at:\n')
            f.write(f'  x = {self.person_x:.2f} m\n')
            f.write(f'  y = {self.person_y:.2f} m\n\n')

            f.write(f'Time to detect person: {self.person_found_time - self.start_time:.2f} s\n')
            f.write(f'Time to reach goal: {self.goal_reached_time - self.person_found_time:.2f} s\n')
            f.write(f'Total mission time: {total_time:.2f} s\n')

        self.get_logger().info(f"Mission report saved at {report_path}")


def main(args=None):
    rclpy.init(args=args)
    node = PersonGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
