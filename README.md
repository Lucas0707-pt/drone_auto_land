# drone_auto_land

This project is designed to enable a drone to autonomously land on a marker using PX4 as an autopilot and ROS2 for communication. It includes a marker detection system that uses a camera feed to detect and track markers in real-time.

## Files in the Project

1. `processes.py`: This script runs the necessary commands for the project in separate terminals. These commands include starting the MicroXRCEAgent, running the image bridge, and starting the PX4 SITL simulation.

2. `marker_detection.py`: This script uses OpenCV to detect and track markers in real-time. It subscribes to the camera feed and publishes the detected marker's position in the camera frame.

3. `frame_converter.py`: This script converts the camera feed from the camera frame to the local frame. It subscribes to the aruco marker's position in the camera frame and publishes the marker's position in the local frame.

4. `controller.py`: This script calculates the drone's desired position based on the marker's position in the local frame. It subscribes to the marker's position in the local frame and publishes the desired position for the drone.

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

3. 1) If you only want to build the drone_auto_land package:

```bash
colcon build --packages-select drone_auto_land
```

4. Source the setup file:

```bash
source install/local_setup.bash
```

5. Run processes with Gazebo GUI (optional) and PX4 SITL simulation (0 for real drone, 1 for simulation):

```bash
ros2 launch drone_auto_land processes.launch.py headless:=1 simulation:=1
```

6. Start marker detection and pose estimation with recording (optional):
    
```bash
ros2 launch drone_auto_land marker_detection.launch.py record:=1
```

7. Start the control node with logging (optional):
    
```bash
ros2 launch drone_auto_land control.launch.py log:=1
```

8. Press Ctrl+C on the terminal running the camera to stop it, the video as output.avi will be saved in the same directory.

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

You need to add your camera matrix and distortion coefficients in the 'camera_parameters' folder.