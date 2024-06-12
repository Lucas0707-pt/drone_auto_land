import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus
import numpy as np
from std_msgs.msg import String

__author__ = "Bruno Silva"
__contact__ = "up201906367@up.pt"

class OffboardLandingController(Node):
    """Node for landing the drone, using Offboard mode, atop the ArUco marker."""

    def __init__(self) -> None:
        super().__init__('offboard_landing_controller')
        
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create QoS profile for publishing the current land state
        qos_profile_state_publisher = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        self.current_land_state_publisher = self.create_publisher(
            String, '/current_land_state', qos_profile_state_publisher)  

        # Create subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        
        self.aruco_pose_local_subscriber = self.create_subscription(
            PoseStamped, '/aruco_pose_local', self.aruco_pose_local_callback, 10)
        
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        self.aruco_pose_camera_subscriber = self.create_subscription(
            PoseStamped, '/aruco_pose_camera', self.aruco_pose_camera_callback, 10)
        # Initialize variables
        self.land_command_sent = False
        self.vehicle_odometry = VehicleOdometry()
        self.desired_x = 0.0
        self.desired_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.desired_z = 0.0
        self.descent_height = 0.2  # Height to descend in z
        self.land_dist_th = 0.5 # Height to land()
        self.goal_z = 0.0
        self.camera_goal_z = 0.0
        self.error_threshold_z = 0.1  # Threshold for z error
        self.error_threshold_xy = 0.15 # Threshold for x and y error
        
        self.state = "Correction" # Correction / Descent / Landing

        # Initialize navigation state
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX

        # Flag to track if setpoint has been published
        self.setpoint_published = False

        # Flag to track if offboard mode has been started
        self.offboard_started = False

        # Gain associated with velocity value
        self.k = 0.5
        self.kz = 1.0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.02, self.timer_callback)

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state

    def vehicle_odometry_callback(self, vehicle_odometry):
        """Callback function for vehicle_odometry topic subscriber."""
        self.vehicle_odometry = vehicle_odometry
        self.current_x = vehicle_odometry.position[0]
        self.current_y = vehicle_odometry.position[1]
        self.current_z = vehicle_odometry.position[2]

    def aruco_pose_local_callback(self, aruco_pose_local):
        """Callback function for aruco_pose_local topic subscriber."""
        self.aruco_pose_local = aruco_pose_local
        self.desired_x = aruco_pose_local.pose.position.x
        self.desired_y = aruco_pose_local.pose.position.y
        self.desired_z = aruco_pose_local.pose.position.z

    def aruco_pose_camera_callback(self, aruco_pose_camera):
        """Callback function for aruco_pose_camera topic subscriber."""
        self.aruco_pose_camera = aruco_pose_camera
        self.current_camera_x = aruco_pose_camera.pose.position.x
        self.current_camera_y = aruco_pose_camera.pose.position.y
        self.current_camera_z = aruco_pose_camera.pose.position.z

        # Calculate velocity setpoints
        self.vx = -self.k * self.current_camera_y
        self.vy = self.k * self.current_camera_x

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def land(self):
        """Command the vehicle to land at its current altitude."""
        if self.current_z is not None: 
            self._logger.info('[L] Landing at the altitude of %.2fm' % abs(self.current_camera_z))
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, param7=float(abs(self.current_z)))
            self.get_logger().info('[L] Land command sent')
            self.land_command_sent = True
            self.state = "Landed"
            self.publish_current_land_state()
            exit(0)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        if self.offboard_started == False:
            self.engage_offboard_mode()
            self.offboard_started = True
            
        self.publish_offboard_control_heartbeat_signal()
        
        # State machine
        if self.state == "Correction" and self.desired_x != 0.0 and self.desired_x != 0.0:
            self.correct_xy_position()
        elif self.state == "Descent":
            self.descend()
        elif self.state == "Landing" and not self.land_command_sent:
            self.land()

        self.publish_current_land_state()

    
    def publish_current_land_state(self):
        """Publish the current land state along with timestamp."""
        msg = String()
        msg.data = f"{self.state}"
        self.current_land_state_publisher.publish(msg)
    

    def correct_xy_position(self):
        """Correct the position of the drone in the horizontal plane."""
        if self.current_x is None or self.current_y is None or self.desired_x is None or self.desired_y is None or self.current_z is None or self.desired_z is None:
            self.get_logger().info("Current x, y, z or desired x, y, z not available.")
            return

        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            if not self.setpoint_published:
                self.get_logger().info("[C] Current position x=%.2fm, y=%.2fm, z=%.2f" % (self.current_x, self.current_y, self.current_z))
                self.get_logger().info("[C] Correcting to x=%.2fm, y=%.2fm, z=%.2f" % (self.desired_x, self.desired_y, self.current_z))
                self.get_logger().info("[C] Current camera position x=%.2fm, y=%.2fm, z=%.2f" % (self.current_camera_x, self.current_camera_y, self.current_camera_z))
                self.get_logger().info("[C] Control velocity x=%.2fm/s, y=%.2fm/s" % (self.vx, self.vy))
                
                # # Generate linear trajectory for correction
                # waypoints = self.generate_linear_trajectory(self.current_x, self.current_y, self.desired_x, self.desired_y, self.current_z, self.current_z, num_points=2)
                
                # # Publish trajectory setpoints
                # for waypoint in waypoints:
                #     self.publish_trajectory_setpoint(waypoint[0], waypoint[1], waypoint[2])
                self.publish_velocity_setpoint(self.vx, self.vy, 0.0)
                self.setpoint_published = False

            # Condition to switch to descent state
            if self.distance_to_desired_position(self.current_x, self.current_y, self.desired_x, self.desired_y) < self.error_threshold_xz:
                self.get_logger().info("[C] Horizontal Error = %.2fm" % (self.distance_to_desired_position(self.current_x, self.current_y, self.desired_x, self.desired_y)))
                self.state = "Descent"
                self.setpoint_published = False

        else:
            self.get_logger().info("Vehicle not in offboard mode.")

    def descend(self):
        """Descend the predefined height."""
        if self.current_z is None or self.descent_height is None or self.desired_z is None or self.current_camera_z is None:
            self.get_logger().info("Current z, camera z, desired z or descent height not available.")
            return
        
        # Adaptive descent height
        if (self.current_camera_z > 5.0):
            self.descent_height = 1.0
        else:
            self.descent_height = 0.2

        # Calculate error in z
        error_z = self.current_camera_z - self.descent_height

  
        if not self.setpoint_published:
            self.goal_z = self.current_z + self.descent_height
            self.camera_goal_z = self.current_camera_z - self.descent_height

            self.get_logger().info("[D] Current position x=%.2fm, y=%.2fm, z=%.2f" % (self.current_x, self.current_y, self.current_camera_z))
            self.get_logger().info("[D] Descending to position x=%.2fm, y=%.2fm, z=%.2f" % (self.current_x, self.current_y, self.camera_goal_z))                       
            ## Generate  z = start_z + t * (end_z - start_z)te linear trajectory for descent
            # waypoints = self.generate_linear_trajectory(self.current_x, self.current_y, self.desired_x, self.desired_y, self.current_z, self.current_z + self.descent_height, num_points=2)
            
            # # Publish trajectory setpoints
            # for waypoint in waypoints:
            #     self.publish_trajectory_setpoint(waypoint[0], waypoint[1], waypoint[2])
            self.setpoint_published = True
        
        error_z = self.current_camera_z - self.camera_goal_z

        self.vz = self.kz * error_z
        self.publish_velocity_setpoint(0.0, 0.0, self.vz)
        self.get_logger().info("[D] Current position x=%.2fm, y=%.2fm, z=%.2f" % (self.current_x, self.current_y, self.current_camera_z))
        self.get_logger().info("[D] Descending to position x=%.2fm, y=%.2fm, z=%.2f" % (self.current_x, self.current_y, self.camera_goal_z))                       
        self.get_logger().info("[D] Control velocity x=%.2fm/s, y=%.2fm/s, z=%.2fm/s" % (0.0, 0.0, self.vz))

        # Condition to switch to landing state
        if abs(self.current_camera_z) < self.land_dist_th:
            self.state = "Landing"

        if abs(error_z) < self.error_threshold_z:
            # Print error information
            self.get_logger().info("[D] Vertical Error = %.2fm" % abs(error_z))
            self.state = "Correction"
            # Reset setpoint_published flag
            self.setpoint_published = False      

    def generate_linear_trajectory(self, start_x, start_y, end_x, end_y, start_z, end_z, num_points):
        """Generate a linear trajectory."""
        trajectory = []
        for i in range(num_points):
            t = i / (num_points - 1)
            x = start_x + t * (end_x - start_x)
            y = start_y + t * (end_y - start_y)
            z = start_z + t * (end_z - start_z)
            trajectory.append((x, y, z))
        return trajectory

    def distance_to_desired_position(self, x1, y1, x2, y2):
        """Calculate the Euclidean distance between two points."""
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def publish_trajectory_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        position_array = np.array([x, y, z], dtype=np.float32)

        # Assign the array to msg.position
        msg.position = position_array
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def generate_linear_trajectory(self, start_x, start_y, end_x, end_y, start_z, end_z, num_points):
        """Generate a linear trajectory."""
        trajectory = []
        for i in range(num_points):
            t = i / (num_points - 1)
            x = start_x + t * (end_x - start_x)
            y = start_y + t * (end_y - start_y)
            z = start_z + t * (end_z - start_z)
            trajectory.append((x, y, z))
        return trajectory

    def distance_to_desired_position(self, x1, y1, x2, y2):
        """Calculate the Euclidean distance between two points."""
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def publish_trajectory_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        position_array = np.array([x, y, z], dtype=np.float32)

        # Assign the array to msg.position
        msg.position = position_array
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_velocity_setpoint(self, vx: float, vy: float, vz: float):
        """Publish the velocity setpoint."""
        msg = TrajectorySetpoint()
        velocity_array = np.array([vx, vy, vz], dtype=np.float32)

        # Assign the array to msg.velocity
        msg.position = np.array([float("nan"), float("nan"), float("nan")], dtype=np.float32)
        msg.velocity = velocity_array
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

def main(args=None) -> None:
    print('Starting offboard landing controller node...')
    rclpy.init(args=args)
    offboard_landing_controller = OffboardLandingController()
    rclpy.spin(offboard_landing_controller)
    offboard_landing_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()