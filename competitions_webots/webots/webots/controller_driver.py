import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from manipulator_interfaces.srv import SetInt32Array


class ControllerDriver(Node):
    def __init__(self):
        super().__init__('Button_node')
        self.pressed = [0, 0, 0, 0]
        self.set_state_srv = self.create_service(SetInt32Array, '~/SetState', self.set_callback)
        self.state_pub = self.create_publisher(Int32MultiArray, '~/State', 10)
        self.allow_pub = self.create_publisher(Bool, '/AllowExecution', 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def set_callback(self, request, response):
        if len(request.data) == 4:
            self.pressed = request.data
            response.success = True
        else:
            response.success = False  
        return response
    
    def timer_callback(self):
        msg_info = Int32MultiArray()
        msg_allow = Bool()
        msg_info.data = self.pressed
        msg_allow.data = not self.pressed[0]
        self.state_pub.publish(msg_info)
        self.allow_pub.publish(msg_allow)

def main():
    rclpy.init()
    driver = ControllerDriver()
    rclpy.spin(driver)
    rclpy.shutdown()

if __name__ == '__main__':
    main()            



