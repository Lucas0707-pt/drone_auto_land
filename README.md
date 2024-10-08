# drone_auto_land (Simulation)

This project enables a simulated drone to autonomously land on a marker using PX4 as an autopilot and ROS2 for marker detection and high-level velocity control. This is done via PX4's [Offboard](https://docs.px4.io/main/en/flight_modes/offboard.html) mode. In the simulation, a virtual camera and markers are used, and the drone’s behavior is tested in a simulated Gazebo environment.

The system simulates a camera feed for marker detection, enabling real-time tracking and positioning corrections. The drone autonomously descends until it lands on the marker.

This branch assumes a simulated drone, using models from a custom Gazebo world, with PX4 running in SITL (Software in the Loop) mode. No actual hardware is required, and the camera feed is simulated directly in Gazebo.

## Files in the Project

1. **`processes.py`**: Starts the simulation environment, including PX4 SITL and simulated image processing. It runs necessary processes like MicroXRCEAgent and commands to start Gazebo with the simulated world.

2. **`marker_detector.py`**: Detects and tracks markers in real-time using traditional marker detection techniques. It subscribes to the simulated camera feed from Gazebo and publishes the detected marker's position in the camera frame.

3. **`marker_detector_open_cv.py`**: An alternative node for marker detection using OpenCV's ArUco marker module, which can be enabled via launch parameters.

4. **`frame_converter.py`**: Converts the simulated camera feed from the camera frame to the drone's body frame. It subscribes to the marker’s position in the camera frame and publishes the position in the drone’s body frame.

5. **`controller.py`**: Computes velocity commands based on the marker's position relative to the drone’s body frame, guiding the drone to land on the marker. It subscribes to the marker’s position in the body frame and publishes velocity commands to PX4 SITL.

6. **`data_logger.py`**: Logs various data points during the simulated landing process, such as the drone's odometry and the marker's position. Data is saved in CSV format for post-analysis.

7. **`data_plot.py`**: Generates 3D plots to visualize the drone’s simulated trajectory relative to the marker. It plots the drone’s odometry and the marker's position.

8. **`velocity_plot.py`**: Generates time-series plots comparing the drone’s velocity setpoints and actual velocities during the simulation.

## Key Differences from Real-Life Setup

- **No `camera_bridge.py`**: Since the camera feed is simulated in Gazebo, there's no need for a `camera_bridge.py` node. The simulated camera is preconfigured in the Gazebo environment.
- **PX4 SITL**: Instead of using a physical drone, PX4 runs in SITL mode, simulating the drone's behavior.
- **Simulated Environment**: The Gazebo world includes a simulated marker and camera, eliminating the need for real sensors or cameras.

## Data Logger Output Files

- **`aruco_pose_camera.csv`**: Logs the simulated ArUco marker’s position in the camera frame.
- **`aruco_pose_drone.csv`**: Logs the simulated ArUco marker’s position in the drone’s body frame.
- **`vehicle_odometry.csv`**: Records the drone’s simulated position, orientation, and velocity.
- **`trajectory_setpoint.csv`**: Logs velocity setpoints sent to the drone during the simulation.
- **`current_land_state.csv`**: Logs the current landing state in the simulated environment.

## How to Run

### Workspace Setup (for Simulation)


1. **Create a new workspace:**

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. **Clone the repository:**

```bash
git clone https://github.com/Lucas0707-pt/drone_auto_land.git
```

3. **Install dependencies and build the workspace:**

```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build
```

3. 1) **If you only want to build the drone_auto_land package:**

```bash
colcon build --packages-select drone_auto_land
```

### Running the Code

1. **Source the setup file:**

```bash
source install/local_setup.bash
```

2. **Run processes with Camera and MicroXRCEAgent:**

The following command launches the necessary processes for running the drone in simulation mode. Set headless:=1 to disable the Gazebo GUI.

```bash
ros2 launch drone_auto_land processes.launch.py
```

3. **Start marker detection and pose estimation with recording (optional):**
    
```bash
ros2 launch drone_auto_land marker_detection.launch.py record:=1 use_opencv:=1
```

4. **Start the control node with logging (optional):**
    
```bash
ros2 launch drone_auto_land control.launch.py log:=1
```

5. **Stop the camera and save the video:**
Press Ctrl+C on the terminal running the camera to stop it, the video as output.avi will be saved in the same directory.

## Dependencies

- Python3
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [PX4](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)
- MicroXRCEAgent
- OpenCV 4.5.4
- numpy
- sensor_msgs
- cv_bridge
- threading

## Note

You need to add your camera matrix and distortion coefficients in the 'camera_parameters' folder.

## Complementary repositories

- Position and Velocity Sliding Mode Controller in the PX4 control architecture:

Custom [repository](https://github.com/BrunoPereira1501/PX4-Autopilot) forked from the [native PX4](https://github.com/PX4/PX4-Autopilot). 

- Custom Models for camera, drone, marker for the custom world "aruco" (must be cloned in /PX4-Autopilot/Tools/simulation/gz):

Custom [repository](https://github.com/BrunoPereira1501/PX4-gazebo-models) forked from the [native gz models repository](https://github.com/PX4/PX4-gazebo-models).
