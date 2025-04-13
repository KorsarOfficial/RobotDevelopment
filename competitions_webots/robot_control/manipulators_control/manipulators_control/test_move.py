#!/usr/bin/env python3
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from .pymoveit2.moveit2 import MoveIt2
from typing import List
from manipulator_interfaces.srv import GoalPose
from geometry_msgs.msg import Pose



class SimpleClient(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('simple_client')
        self.cli_joint = self.create_client(GoalPose, 'MoveJ')

        while not self.cli_joint.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available")

        self.cli_linear = self.create_client(GoalPose, 'MoveL')

        while not self.cli_linear.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available")

        self.req = GoalPose.Request()
    
    def send_movej(self, pose):
        self.req.position = [pose.position.x,pose.position.y,pose.position.z]
        self.req.orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        self.future = self.cli_joint.call_async(self.req)

        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()
    def send_movel(self, pose):
        self.req.position = [pose.position.x,pose.position.y,pose.position.z]
        self.req.orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        self.future = self.cli_linear.call_async(self.req)

        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()
    
def main():
    client = SimpleClient()

    start_pose = Pose()

    start_pose.position.x = 0.291
    start_pose.position.y = 0.0
    start_pose.position.z = 0.188

    # pose_req.orientation.x = -8.59*10**(-5)
    # pose_req.orientation.y = -4.9755*10**(-5)
    response = False
    start_pose.orientation.x = 0.0148
    start_pose.orientation.y = 1.0
    start_pose.orientation.z = 0.0
    start_pose.orientation.w = 0.00147
    response = client.send_movej(start_pose)

    pose1 = Pose()
    pose1.position.x = 0.287
    pose1.position.y = -0.128
    pose1.position.z = 0.188

    pose1.orientation.x = 0.0148
    pose1.orientation.y = 1.0
    pose1.orientation.z = 0.0
    pose1.orientation.w = 0.00147

    pose2 = Pose()
    pose2.position.x = 0.287
    pose2.position.y = 0.128
    pose2.position.z = 0.188

    pose2.orientation.x = 0.0148
    pose2.orientation.y = 1.0
    pose2.orientation.z = 0.0
    pose2.orientation.w = 0.00147  

    while rclpy.ok():
        response = client.send_movel(pose1)
        response = client.send_movel(pose2)

        



    client.get_logger().info(response.success)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()