#!/usr/bin/env python3
from threading import Thread
import math, time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from .pymoveit2.moveit2 import MoveIt2
from typing import List
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from manipulator_interfaces.srv import GoalPose
from manipulator_interfaces.srv import JointPosition
from manipulator_interfaces.srv import GripperDownPose


def quat_mul(q_1, q_2):
    x1, y1, z1, w1 = q_1
    x2, y2, z2, w2 = q_2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2]

def action_availiable(node):
    return (node.moveit2._MoveIt2__move_action_client.server_is_ready() and
            node.moveit2._plan_kinematic_path_service.service_is_ready() and
            node.moveit2._plan_cartesian_path_service.service_is_ready() and
            node.moveit2._MoveIt2__follow_joint_trajectory_action_client.server_is_ready()) 


class RobotControl(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('manipulator_control')
        callback_group = ReentrantCallbackGroup()

        self.allow_execution = True
        self.allow_execution_general = True

        joint_names = ["Joint1", "Joint2","Joint3", "Joint4", "Joint5", "Joint6"]
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name="Base_link",
            end_effector_name="tool",
            group_name="arm1650_group",
            callback_group=callback_group,
            execute_via_moveit=False,
            follow_joint_trajectory_action_name='/arm1650_controller/follow_joint_trajectory'
        )        

        self.moveJ_srv = self.create_service(GoalPose, '/ARM165/MoveJ', self.moveJ_callback, callback_group=callback_group)
        self.moveL_srv = self.create_service(GoalPose, '/ARM165/MoveL', self.moveL_callback, callback_group=callback_group)
        self.moveJoint_srv = self.create_service(JointPosition, '/ARM165/MoveJoint', self.moveJoint_callback, callback_group=callback_group)
        self.moveJ_cyl_srv = self.create_service(GripperDownPose, '/ARM165/MoveJCyl', self.moveJCyl_callback, callback_group=callback_group)
        self.moveL_cyl_srv = self.create_service(GripperDownPose, '/ARM165/MoveLCyl', self.moveLCyl_callback, callback_group=callback_group)
        
        self.allow_sub_1 = self.create_subscription(Bool, '/ARM165/AllowExecution', self.allow_execution_callback, 10, callback_group=callback_group)
        self.allow_sub_2 = self.create_subscription(Bool, '/AllowExecution', self.allow_execution_callback_general, 10, callback_group=callback_group)

        self.vacuum_gripper_srv = self.create_service(SetBool, '/ARM165/vacuum_gripper/turn_on', self.vacuum_gripper_callback, callback_group=callback_group)
        self.suction_cup_pub = self.create_publisher(Bool, '/ARM165/vacuum_gripper/turn_on', 1)

    def moveJ_callback(self, request, response):
        if self.allow_execution and self.allow_execution_general:
            goal_pose = request
            goal_position = [goal_pose.position[0], goal_pose.position[1], goal_pose.position[2]]
            goal_orientation = [goal_pose.orientation[0], goal_pose.orientation[1], goal_pose.orientation[2], goal_pose.orientation[3]]
            self.moveit2.move_to_pose(position=goal_position, quat_xyzw=goal_orientation, cartesian=False, tolerance_position=0.001, tolerance_orientation=0.001)
            response.success = self.moveit2.wait_until_executed()
        else:
            response.success = False
        return response
        
    def moveL_callback(self, request, response):
        if self.allow_execution and self.allow_execution_general:
            goal_pose = request
            goal_position = [goal_pose.position[0], goal_pose.position[1], goal_pose.position[2]]
            goal_orientation = [goal_pose.orientation[0], goal_pose.orientation[1], goal_pose.orientation[2], goal_pose.orientation[3]]
            self.moveit2.move_to_pose(position=goal_position, quat_xyzw=goal_orientation, cartesian=True)
            response.success = self.moveit2.wait_until_executed()
        else:
            response.success = False
        return response
    
    def moveJoint_callback(self, request, response):
        if self.allow_execution and self.allow_execution_general:
            goal_position = request.joint_position
            self.moveit2.move_to_configuration(joint_positions=goal_position, tolerance=0.001)
            response.success = self.moveit2.wait_until_executed()
        else:
            response.success = False
        return response

    def moveJCyl_callback(self, request, response):
        if self.allow_execution and self.allow_execution_general:
            goal_pose = request
            goal_position = [math.cos(goal_pose.position[0])*goal_pose.position[1], math.sin(goal_pose.position[0])*goal_pose.position[1], goal_pose.position[2]]
            goal_orientation = quat_mul([0, 1, 0, 0], [0, 0, math.sin((goal_pose.gripper_angle)/2), math.cos((goal_pose.gripper_angle)/2)])
            self.moveit2.move_to_pose(position=goal_position, quat_xyzw=goal_orientation, cartesian=False, tolerance_position=0.001, tolerance_orientation=0.001)
            response.success = self.moveit2.wait_until_executed()
        else:
            response.success = False
        return response

    def moveLCyl_callback(self, request, response):
        if self.allow_execution and self.allow_execution_general:
            goal_pose = request
            goal_position = [math.cos(goal_pose.position[0])*goal_pose.position[1], math.sin(goal_pose.position[0])*goal_pose.position[1], goal_pose.position[2]]
            goal_orientation = quat_mul([0, 1, 0, 0], [0, 0, math.sin((goal_pose.gripper_angle)/2), math.cos((goal_pose.gripper_angle)/2)])
            self.moveit2.move_to_pose(position=goal_position, quat_xyzw=goal_orientation, cartesian=True)
            response.success = self.moveit2.wait_until_executed()
        else:
            response.success = False
        return response

    def allow_execution_callback(self, msg):
        self.reset_execution(msg)
        self.allow_execution = msg.data

    def allow_execution_callback_general(self, msg):
        self.reset_execution(msg)
        self.allow_execution_general = msg.data

    def reset_execution(self, msg):
        if not msg.data and (self.moveit2._MoveIt2__is_motion_requested or self.moveit2._MoveIt2__is_executing):
            self.moveit2.goal_handle.cancel_goal_async()
            self.moveit2.force_reset_executing_state()
            

    def vacuum_gripper_callback(self, request : SetBool.Request, response : SetBool.Response):
        if self.allow_execution and self.allow_execution_general:
            msg = Bool()
            msg.data = request.data
            self.suction_cup_pub.publish(msg)
            response.success = True
        else:
            response.success = False
        return response


def main():
    robot_control_node = RobotControl()

    while not action_availiable(robot_control_node):
        continue
    executor = MultiThreadedExecutor(2)

    executor.add_node(robot_control_node)

    executor_thread = Thread(target=executor.spin, daemon=True)

    executor_thread.start()
    rate = robot_control_node.create_rate(5)
    time.sleep(1)
    robot_control_node.moveit2.move_to_configuration(joint_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], tolerance=0.001)
    robot_control_node.moveit2.wait_until_executed()

    while rclpy.ok() and 'controller_manager' in robot_control_node.get_node_names():
        rate.sleep()

    rclpy.shutdown()

    executor_thread.join()


if __name__ == "__main__":
    main()




