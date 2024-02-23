#!/usr/bin/env python3

import subprocess
import time

def main():
    # Commands to run in separate terminals
    micro_XRCE_agent = "MicroXRCEAgent udp4 -p 8888"
    camera_bridge = "ros2 run ros_gz_image image_bridge /camera"
    px4_sitl_simulation = "cd ../PX4-Autopilot && PX4_GZ_WORLD=aruco make px4_sitl gz_x500_depth ./build/px4_sitl_default/bin/px4"

    # Run the commands in separate terminals
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", micro_XRCE_agent + "; exec bash"])
    time.sleep(1)
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", camera_bridge + "; exec bash"])
    time.sleep(1)
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", px4_sitl_simulation + "; exec bash"])
    time.sleep(1)

if __name__ == "__main__":
    main()