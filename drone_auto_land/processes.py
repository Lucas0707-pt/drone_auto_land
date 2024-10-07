import subprocess
import time
import rclpy
from rclpy.node import Node

class ProcessesNode(Node):
    def __init__(self):
        super().__init__('processes')
        self.declare_parameter('headless', 1)
        self.headless = bool(self.get_parameter('headless').get_parameter_value().integer_value)
        self.main()

    def main(self):
        # Commands to run in separate terminals
        micro_XRCE_agent_sim = ["MicroXRCEAgent", "udp4", "-p", "8888"]
        camera_bridge_sim = ["ros2", "run", "ros_gz_image", "image_bridge", "/camera"]
        px4_sitl_sim = ["gnome-terminal", "--tab", "--","bash", "-c", "cd ../PX4-Autopilot && " + ("HEADLESS=1 " if self.headless else "") + "PX4_GZ_WORLD=aruco make px4_sitl gz_x500_depth", "; exec bash"]

        # Run the commands
        subprocess.Popen(micro_XRCE_agent_sim)
        time.sleep(1)
        subprocess.Popen(camera_bridge_sim)
        time.sleep(1)
        subprocess.run(px4_sitl_sim)


def main(args=None):
    rclpy.init(args=args)
    processes_node = ProcessesNode()
    processes_node.main()
    rclpy.spin(processes_node)
    processes_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
