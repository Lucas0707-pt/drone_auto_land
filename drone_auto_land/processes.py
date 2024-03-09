#!/usr/bin/env python3

import subprocess
import time
import rclpy
from rclpy.node import Node

class ProcessesNode(Node):
    def __init__(self):
        super().__init__('processes')
        self.declare_parameter('headless', 0)  # Declare the 'headless' parameter as an integer with a default value of 0
        self.headless = bool(self.get_parameter('headless').get_parameter_value().integer_value)
        self.main()

    def main(self):
        # Commands to run in separate terminals
        micro_XRCE_agent = "MicroXRCEAgent udp4 -p 8888"
        camera_bridge = "ros2 run ros_gz_image image_bridge /camera"
        px4_sitl_simulation = "cd ../PX4-Autopilot && "
        if self.headless:
            px4_sitl_simulation += "HEADLESS=1 "
        px4_sitl_simulation += "PX4_GZ_WORLD=aruco make px4_sitl gz_x500_depth ./build/px4_sitl_default/bin/px4"

        # Run the commands in separate terminals
        subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", micro_XRCE_agent + "; exec bash"])
        time.sleep(1)
        subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", camera_bridge + "; exec bash"])
        time.sleep(1)
        subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", px4_sitl_simulation + "; exec bash"])
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    processes_node = ProcessesNode()
    rclpy.spin(processes_node)
    processes_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
