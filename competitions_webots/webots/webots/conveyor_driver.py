import rclpy
from std_msgs.msg import Float32

class ConveyorBeltDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__robot_name = self.__robot.getName()
        self.__timestep = int(self.__robot.getBasicTimeStep())

        self.__belt_motor = self.__robot.getDevice('belt_motor')
        self.__belt_motor.setPosition(float('inf'))
        self.__belt_motor.setVelocity(0.0)

        rclpy.init(args=None)
        self.__node = rclpy.create_node(f"{self.__robot.getName()}_driver")
        self.__node.get_logger().info(f"Conveyor Belt Driver for {self.__robot.getName()} initialized")

        self.target_speed = Float32()
        self.target_speed.data = 0.0
        self.__node.create_subscription(Float32, f"{self.__robot_name}/cmd_vel", self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, msg: Float32):
        self.target_speed = msg
        self.__belt_motor.setVelocity(self.target_speed.data)        

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)