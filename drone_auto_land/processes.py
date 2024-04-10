import subprocess
import time
import rclpy
from rclpy.node import Node

class ProcessesNode(Node):
    def __init__(self):
        super().__init__('processes')
        self.declare_parameter('headless', 0) 
        self.declare_parameter('simulation', 1)
        self.headless = bool(self.get_parameter('headless').get_parameter_value().integer_value)
        self.simulation = bool(self.get_parameter('simulation').get_parameter_value().integer_value)
        self.main()

    def main(self):
        # Commands to run in separate terminals
        micro_XRCE_agent_sim = ["MicroXRCEAgent", "udp4", "-p", "8888"]
        camera_bridge_sim = ["ros2", "run", "ros_gz_image", "image_bridge", "/camera"]
        px4_sitl_sim = ["gnome-terminal", "--tab", "--","bash", "-c", "cd ../PX4-Autopilot && " + ("HEADLESS=1 " if self.headless else "") + "PX4_GZ_WORLD=aruco make px4_sitl gz_x500_depth", "; exec bash"]

        micro_XRCE_agent_drone = ["sudo", "MicroXRCEAgent", "serial", "--dev", "/dev/serial0", "-b", "921600"]
        camera_bridge_drone = ["ros2", "run", "v4l2_camera", "v4l2_camera_node"]

        # Run the commands in separate terminals
        if self.simulation:
            subprocess.Popen(micro_XRCE_agent_sim)
            time.sleep(1)
            subprocess.Popen(camera_bridge_sim)
            time.sleep(1)
            subprocess.run(px4_sitl_sim)
        else:
            subprocess.Popen(micro_XRCE_agent_drone)
            time.sleep(1)
            subprocess.Popen(camera_bridge_drone)
        time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    processes_node = ProcessesNode()
    processes_node.main()
    rclpy.spin(processes_node)
    processes_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
