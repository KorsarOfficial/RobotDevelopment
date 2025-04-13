import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray

class LampDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__led = [None]*4
        self.__led[0] = self.__robot.getDevice('led0')
        self.__led[1] = self.__robot.getDevice('led1')
        self.__led[2] = self.__robot.getDevice('led2')
        self.__led[3] = self.__robot.getDevice('led3')

        for i in range(4):
            self.__led[i].set(0)

        self.__lamp_array = Int8MultiArray()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('lamp_driver')
        self.__node.create_subscription(Int8MultiArray, 'lamp_topic', self.__lamp_callback, 1)

    def __lamp_callback(self, array:Int8MultiArray):
        # led_state = []
        # for state in array.data:
        #     led_state.append(state)
        self.__lamp_array = array

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        for i in range(4):
            if self.__lamp_array.data[i] == 0:
                self.__led[i].set(0)
            if self.__lamp_array.data[i] == 1:
                self.__led[i].set(255)

            



