import rclpy
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
import time
from concurrent.futures import ThreadPoolExecutor


ROBOTS = ['Palletizer', 'Black5dof', 'Black5dofVacuum', 'PalletizerGripper']

class SimulationSupervisor:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__robot_name = self.__robot.getName()
        self.__timestep = int(self.__robot.getBasicTimeStep())
        rclpy.init(args=None)
        self.__node = rclpy.create_node("simulation_supervisor")
        self.__robot_indices = []
        self.__node_list = []
        for i in range(1000):
            obj = self.__robot.getFromId(i)
            if obj:
                self.__node_list.append(obj)
                if obj.getTypeName() in ROBOTS:
                    self.__robot_indices.append(len(self.__node_list) - 1)
        self.toggle_ws_srv = self.__node.create_service(SetBool, '~/ToggleWs', self.__toggle_ws_callback)

        self.blinked = False
        self.start_time = time.time()
        self.set_ws_transparency(0.8)

        # self.__node.get_logger().info("Simulation Supervisor initialized")
        
    def __toggle_ws_callback(self, request, response):
        ws_trans = 1
        if request.data:
            ws_trans = 0.8
        self.set_ws_transparency(ws_trans)
        response.success = True
        return response

    def set_ws_transparency(self, value):
        for ind in self.__robot_indices:
            self.__node_list[ind].getField("wsTransparency").setSFFloat(float(value))

    def step(self):
        if not self.blinked and self.start_time + 3 <= time.time():
            self.blinked = True
            self.set_ws_transparency(1)

        rclpy.spin_once(self.__node, timeout_sec=0)