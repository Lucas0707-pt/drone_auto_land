# drone_auto_land

This project is designed to enable a drone to autonomously land on a marker using PX4 as an autopilot and ROS2 for communication. It includes a marker detection system that uses a camera feed to detect and track markers in real-time.

## Files in the Project

1. `processes.py`: This script runs the necessary commands for the project in separate terminals. These commands include starting the MicroXRCEAgent, running the image bridge, and starting the PX4 SITL simulation.

2. `uav_camera_sim.py`: This script is a ROS2 node that subscribes to the camera feed, performs marker detection, and records the video output.

## How to Run

1. Ensure that you have ROS2 Humble and PX4 installed on your system.

2. Source the setup file:

```bash
source /opt/ros/humble/setup.bash
```

# ros2 launch drone_auto_land drone_auto_land.launch.py say that this is the command to run the project

3. Run processes:

```bash
ros2 launch drone_auto_land drone_auto_land.launch.py
```

4. Start the camera feed and marker detection:
    
```bash
ros2 launch drone_auto_land camera_sim.launch.py
```

## Dependencies

- Python3
- ROS2
- PX4
- OpenCV 4.5.4.60: Install with `pip install opencv-contrib-python==4.5.4.60`
- numpy
- sensor_msgs
- cv_bridge
- threading

## Note

You need to add your camera matrix and distortion coefficients in the `uav_camera_sim.py` file.




