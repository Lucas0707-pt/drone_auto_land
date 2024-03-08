# drone_auto_land

This project is designed to enable a drone to autonomously land on a marker using PX4 as an autopilot and ROS2 for communication. It includes a marker detection system that uses a camera feed to detect and track markers in real-time.

## Files in the Project

1. `processes.py`: This script runs the necessary commands for the project in separate terminals. These commands include starting the MicroXRCEAgent, running the image bridge, and starting the PX4 SITL simulation.

2. `uav_camera_sim.py`: This script is a ROS2 node that subscribes to the camera feed, performs marker detection, and records the video output.

## How to Run

1. Create a new workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone the repository:

```bash
git clone https://github.com/Lucas0707-pt/drone_auto_land.git
```

3. Build the workspace:

```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build
```

3.1 If you only want to build the drone_auto_land package:

```bash
colcon build --packages-select drone_auto_land
```

4. Source the setup file:

```bash
source install/local_setup.bash
```

5. Run processes:

```bash
ros2 launch drone_auto_land drone_auto_land.launch.py
```

6. Start the camera feed and marker detection:
    
```bash
ros2 launch drone_auto_land camera_sim.launch.py
```

7. Press Ctrl+C on the terminal running the camera to stop it, the video as output.avi will be saved in the same directory.

## Dependencies

- Python3
- ROS2 Humble
- PX4
- MicroXRCEAgent
- OpenCV 4.5.4
- numpy
- sensor_msgs
- cv_bridge
- threading

## Note

You need to add your camera matrix and distortion coefficients in the `uav_camera_sim.py` file.




