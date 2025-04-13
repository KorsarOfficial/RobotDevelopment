import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from manipulator_interfaces.srv import SetInt32


class ButtonDriver(Node):
    def __init__(self):
        super().__init__('Button_node')
        self.pressed = 0
        self.set_state_srv = self.create_service(SetInt32, '~/SetState', self.set_callback)
        self.state_pub = self.create_publisher(Int32, '~/State', 10)
        if self.get_namespace() == '/angle_controller':
            self.allow_pub = self.create_publisher(Bool, '/angle/AllowExecution', 10)
        elif self.get_namespace() == '/palletizer_controller':
            self.allow_pub = self.create_publisher(Bool, '/palletizer/AllowExecution', 10)
        else: 
            self.allow_pub = None
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def set_callback(self, request, response):
        self.pressed = request.data
        response.success = True
        return response
    
    def timer_callback(self):
        msg_info = Int32()
        msg_info.data = self.pressed
        self.state_pub.publish(msg_info)
        if self.allow_pub:
            msg_allow = Bool()
            msg_allow.data = not self.pressed
            self.allow_pub.publish(msg_allow)


def main():
    rclpy.init()
    driver = ButtonDriver()
    rclpy.spin(driver)
    rclpy.shutdown()

if __name__ == '__main__':
    main()            



