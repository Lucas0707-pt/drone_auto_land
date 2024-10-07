# drone_auto_land

This project enables a drone to autonomously land on a marker using PX4 as an autopilot and ROS2 for marker detection and high-level velocity control. This interaction is facilitated by PX4's flight mode [Offboard](https://docs.px4.io/main/en/flight_modes/offboard.html). The system includes a marker detection system that utilizes a camera feed to detect and track markers in real-time, performing position correction and altitude descent.

Several branches of this project are adapted for real drones, assuming the drone is equipped with a Holybro Pixhawk 4 running PX4, along with sensor integration and a USB interface camera (e.g., Logitech C920). A companion computer running [Ubuntu 22.04](https://ubuntu.com/download/raspberry-pi) is required, with the dependencies outlined below. Other branches are adapted for simulation environments using the same dependencies and additional models, which are mentioned in the "Complementary repositories" section.

## Files in the Project

1. **`processes.py`**: Runs commands for the project in separate terminals, including starting the MicroXRCEAgent, running the image bridge, and launching the PX4 SITL simulation.

2. **`marker_detector.py`**: Detects and tracks markers in real-time using OpenCV. It subscribes to the camera feed and publishes the detected marker's position in the camera frame.

3. **`marker_detector_open_cv.py`**: An alternative marker detection node using OpenCV, which can be enabled via launch parameters.

4. **`frame_converter.py`**: Converts the camera feed from the camera frame to the drone's body frame. It subscribes to the marker’s position in the camera frame and publishes the position in the drone’s body frame.

5. **`controller.py`**: Calculates the drone's desired velocity based on the marker’s position in the body frame, targeting the origin of the coordinate system. It subscribes to the marker’s position in the body frame and publishes the velocity commands for the PX4 autopilot.

6. **`data_logger.py`**: Logs various data during the landing process, such as the drone's odometry, the marker’s position, and the landing sequence state. Data is saved into CSV files for further analysis.

7. **`data_plot.py`**: Generates 3D plots to visualize the drone’s trajectory during the landing procedure, plotting the marker's position relative to the drone and the drone's odometry.

8. **`velocity_plot.py`**: Generates time-series plots comparing the drone’s velocity setpoints and actual velocities. It highlights transitions between states like correction, descent, and landing.

## Data Logger Output Files

- **`aruco_pose_camera.csv`**: Records the ArUco marker’s position in the camera frame.
- **`aruco_pose_drone.csv`**: Records the ArUco marker’s position in the drone’s body frame.
- **`vehicle_odometry.csv`**: Records the drone’s position, orientation, and velocity.
- **`trajectory_setpoint.csv`**: Logs velocity setpoints sent to the drone.
- **`current_land_state.csv`**: Logs the current state of the landing process.

## How to Run

### Workspace Setup (after PX4 and ROS2 setup)

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

2. **Run processes with Gazebo GUI (optional) and PX4 SITL simulation:**

The following command launches the necessary processes for running the drone in simulation mode. Set headless:=1 to disable the Gazebo GUI.

```bash
ros2 launch drone_auto_land processes.launch.py headless:=1
```

3. **Start marker detection and pose estimation with recording (optional):**
    
```bash
ros2 launch drone_auto_land marker_detection.launch.py record:=1 use_opencv:=true
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