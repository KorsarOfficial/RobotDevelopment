#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from manipulator_interfaces.srv import GripperDownPose, GoalPose, GripperCmd
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Int8MultiArray, Float32
from threading import Thread
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# Импортируем класс для движения паллетайзера из palletizer_test.py
from .palletizer_test import SimpleClient as PalSimpleClient
# Импортируем класс для движения углового из angle_test.py
from .angle_test import SimpleClient as AngSimpleClient
import copy
import numpy as np
from .angle_test import Pose_msg_create as Pose_msg_create_ang
from .palletizer_test import Pose_msg_create as Pose_msg_create_pal
from .angle_robot_control import quat_mul
import math


class LampsConveyor(Node):
    ''' Класс для управления лампами и конвейером. '''
    def __init__(self):
        super().__init__('simple_client_lamps_conveyor')
        # Создаем объекты для передачи массива для ламп через топик:
        self.left_lamp_pub = self.create_publisher(Int8MultiArray, '/Left_Lamp_driver/lamp_topic', 1)
        self.right_lamp_pub = self.create_publisher(Int8MultiArray, '/Right_Lamp_driver/lamp_topic', 1)
        # Создаем объект для передачи скорости конвейера через топик:
        # self.conveyor_pub = self.create_publisher(Float32, '/Conveyor/cmd_vel', 1)
    
    def lamp_turn_on(self, lamp_object, state):
        ''' Передача в ROS-топик массива значений state лампы. '''
        msg = Int8MultiArray()
        msg.data = state
        lamp_object.publish(msg)
    
    # def move_conveyor(self, velocity):
    #     ''' Передача в ROS-топик скорости для конвейера. '''
    #     msg = Float32()
    #     msg.data = velocity
    #     self.conveyor_pub.publish(msg)
    
    def wait_for_secs(self, time_secs: float):
        ''' Ожидание заданного количества секунд. '''
        timer = self.create_timer(time_secs, self.__timer_callback, clock=self.get_clock())
        while not timer.is_ready():
            pass
    
    def __timer_callback(self):
        self.get_logger().info(f'Timer expired!')


def relative_position(cube_size, target_xyz, rel_xyz, z_angle):
    new_coords = [x - p for x, p in zip(target_xyz, rel_xyz)]
    rot_matrix = np.array([[np.cos(z_angle), -np.sin(z_angle)],
                           [np.sin(z_angle), np.cos(z_angle)]])
    new_vec = np.matmul(rot_matrix, np.array([new_coords[0], new_coords[1]]))
    new_coords[0], new_coords[1] = new_vec[0], new_vec[1]
    new_coords[2] += cube_size/2 -0.005
    return new_coords


def main():
    rclpy.init()
    palletizer = PalSimpleClient()
    angle = AngSimpleClient()
    client = LampsConveyor()

    executor = MultiThreadedExecutor(3)
    executor.add_node(palletizer)
    executor.add_node(angle)
    executor.add_node(client)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    cube_size = 0.025 # Высота кубиков в сцене cube_size единиц
    angle_xyz  = [0.225, 0.0, 0.74]

    first_cube_xyz  = [0.31, 0.22, 0.74]
    second_cube_xyz = [0.23, 0.22, 0.74]
    third_cube_xyz = [0.15, 0.22, 0.74]

    target_pose_angle = Pose_msg_create_ang([0.2, 0.0, cube_size/2 + 0.14], [0.0, 1.0, 0.0, 0.0])

    obj_pose_palletizer = [[0.25, 0.0, cube_size], 0.0]
    target_pose_palletizer = [[0.0, 0.2, cube_size*4], 0.0]
    
    obj_poses = [relative_position(cube_size, xyz, angle_xyz, math.pi) for xyz in [first_cube_xyz,
                                                                        second_cube_xyz,
                                                                        third_cube_xyz]]

    # Перемещаем кубики между заданными точками с помощью присоски (реализовано в функции grasp_and_move):
    green_blue_red_leds = {-1: [0, 0, 0, 0], 0: [1,0,0,0], 1: [0,1,0,0], 2: [0,0,0,1]}

    client.lamp_turn_on(client.left_lamp_pub, green_blue_red_leds[0])
    client.lamp_turn_on(client.right_lamp_pub, green_blue_red_leds[-1])
    angle.grasp_and_move(Pose_msg_create_ang(obj_poses[0], quat_mul([0.0, 1.0, 0.0, 0.0], [0, 0, math.sin(math.pi/4), math.cos(math.pi/4)])), target_pose_angle)
    angle.pick(Pose_msg_create_ang(obj_poses[1], quat_mul([0.0, 1.0, 0.0, 0.0], [0, 0, math.sin(math.pi/4), math.cos(math.pi/4)])))
    client.lamp_turn_on(client.left_lamp_pub, green_blue_red_leds[1])
    client.lamp_turn_on(client.right_lamp_pub, green_blue_red_leds[0])
    palletizer.grasp_and_move(obj_pose_palletizer, target_pose_palletizer)
    angle.place(target_pose_angle)
    angle.pick(Pose_msg_create_ang(obj_poses[2], quat_mul([0.0, 1.0, 0.0, 0.0], [0, 0, math.sin(math.pi/4), math.cos(math.pi/4)])))
    client.lamp_turn_on(client.left_lamp_pub, green_blue_red_leds[2])
    client.lamp_turn_on(client.right_lamp_pub, green_blue_red_leds[1])
    palletizer.grasp_and_move(obj_pose_palletizer, target_pose_palletizer)
    angle.place(target_pose_angle)
    client.lamp_turn_on(client.left_lamp_pub, green_blue_red_leds[-1])
    angle.send_movej(Pose_msg_create_ang([0.0, -0.15, 0.2], quat_mul([0.0, 1.0, 0.0, 0.0], [0, 0, math.sin(math.pi/4), math.cos(math.pi/4)])))
    client.lamp_turn_on(client.right_lamp_pub, green_blue_red_leds[2])
    palletizer.grasp_and_move(obj_pose_palletizer, target_pose_palletizer)
    client.lamp_turn_on(client.right_lamp_pub, green_blue_red_leds[-1])
        
    palletizer.destroy_node()
    angle.destroy_node()
    client.destroy_node()
    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()
