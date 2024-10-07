import subprocess
import time
import rclpy
from rclpy.node import Node

class ProcessesNode(Node):
    def __init__(self):
        super().__init__('processes')
        self.main()

    def main(self):
        micro_XRCE_agent_drone = ["sudo", "MicroXRCEAgent", "serial", "--dev", "/dev/serial0", "-b", "921600"]
        subprocess.Popen(micro_XRCE_agent_drone)
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
