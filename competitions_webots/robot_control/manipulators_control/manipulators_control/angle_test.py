#!/usr/bin/env python3
from threading import Thread
import copy
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from typing import List
from manipulator_interfaces.srv import GoalPose, GripperCmd, JointPosition
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import time



class SimpleClient(Node):
    def __init__(self):
        super().__init__('angle_simple_client')
        self.cli_joint = self.create_client(GoalPose, '/ARM165/MoveJ')
        # self.vacuum_gripper_pub = self.create_publisher(Bool, '/angle/suction_cup/turn_on', qos_profile=1)
        self.vacuum_gripper_turn = self.create_client(SetBool, '/ARM165/vacuum_gripper/turn_on')

        while not self.cli_joint.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available")

        self.cli_linear = self.create_client(GoalPose, '/ARM165/MoveL')

        while not self.cli_linear.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available")

        self.req = GoalPose.Request()


        self.initial_joint_pose = Pose()
        self.initial_joint_pose.position.x = 0.19522
        self.initial_joint_pose.position.y = 0.0
        self.initial_joint_pose.position.z = 0.1936

        self.initial_joint_pose.orientation.x = 0.0
        self.initial_joint_pose.orientation.y = 1.0
        self.initial_joint_pose.orientation.z = 0.0
        self.initial_joint_pose.orientation.w = 0.0

    def move_to_init(self):
        if not self.send_movej(self.initial_joint_pose):
            self.get_logger().error("Failed to move initial pose")

    def send_movej(self, pose : Pose):
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
    
    def gripper_turn(self, state):
        msg = SetBool.Request()
        msg.data = state
        self.vacuum_gripper_turn.call_async(msg)

    def pick(self, obj_pose : Pose):
        z_offset = 0.0
        start_pose = copy.deepcopy(obj_pose)
        start_pose.position.z += z_offset

        response = False

        response = self.send_movej(start_pose)

        if not response:
            print("Start pose not allowed")
            return
        
        if not self.send_movel(obj_pose):
            print("Object pose not allowed")
            return
        
        self.gripper_turn(True)

        time.sleep(1)

        start_pose.position.z += 0.1

        if not self.send_movel(start_pose):
            return

    def place(self, target_pose : Pose):
        if not self.send_movej(target_pose):
            print("Target pose not allowed")
            return

        target_pose.position.z -= 0.1
        if not self.send_movel(target_pose):
            print("Target pose not allowed")
            return

        self.gripper_turn(False) 

        time.sleep(0.5)
        target_pose.position.z += 0.1
        if not self.send_movel(target_pose):
            print("Target pose not allowed")
            return

        # self.move_to_init()

    def grasp_and_move(self, obj_pose : Pose, target_pose : Pose):
        self.pick(obj_pose)
        self.place(target_pose)


def Pose_msg_create(position, orientation):
    msg = Pose()
    msg.position.x = position[0]
    msg.position.y = position[1]
    msg.position.z = position[2]

    msg.orientation.x = orientation[0]
    msg.orientation.y = orientation[1]
    msg.orientation.z = orientation[2]
    msg.orientation.w = orientation[3]

    return msg


def main():
    rclpy.init()
    client = SimpleClient()

    first_cube_pose = Pose_msg_create([1.26,-0.02, 0.13], [0.0, 1.0, 0.0, 0.0])
    second_cube_pose = Pose_msg_create([0.269, 0.01, 0.0127], [0.0, 1.0, 0.0, 0.0])
    third_cube_pose = Pose_msg_create([0.269, 0.114, 0.0127], [0.0, 1.0, 0.0, 0.0])

    # first_cube_target_pose = Pose_msg_create([0.0, 0.294, 0.0255], [0.0, 0.0, 0.0, 1.0])
    # second_cube_target_pose = Pose_msg_create([0.0, 0.294, 0.05], [0.0, 0.0, 0.0, 1.0])
    # third_cube_target_pose = Pose_msg_create([0.0, 0.294, 0.075], [0.0, 0.0, 0.0, 1.0])

    first_cube_target_pose = Pose_msg_create([0.03, -1.11, 0.08], [0.0, 1.0, 0.0, 0.0])
    second_cube_target_pose = Pose_msg_create([0.0, 0.2379, 0.175], [0.707, -0.707, 0.0, 0.0])
    third_cube_target_pose = Pose_msg_create([0.0, 0.2379, 0.175], [0.707, -0.707, 0.0, 0.0])

    obj_poses = [first_cube_pose, second_cube_pose, third_cube_pose]
    target_poses = [first_cube_target_pose, second_cube_target_pose, third_cube_target_pose]

    # while rclpy.ok():
    #     response = client.send_movel(pose1)
    #     response = client.send_movel(pose2)

    client.grasp_and_move(obj_poses[0], target_poses[0])

        
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()