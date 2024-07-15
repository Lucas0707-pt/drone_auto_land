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
        
        self.aruco_pose_drone = self.create_subscription(
            PoseStamped, 'aruco_pose_drone', self.aruco_pose_drone_callback, 10)
        
        # Initialize variables
        self.land_command_sent = False
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.drone_z = 0.0
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
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state

    def aruco_pose_drone_callback(self, aruco_pose_drone):
        """Callback function for aruco_pose_drone topic subscriber."""
        self.drone_x = aruco_pose_drone.pose.position.x
        self.drone_y = aruco_pose_drone.pose.position.y
        self.drone_z = aruco_pose_drone.pose.position.z

        # Calculate velocity setpoints
        self.vx = -self.k * self.drone_y
        self.vy = self.k * self.drone_x

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
            self._logger.info('[L] Landing at the altitude of %.2fm' % abs(self.drone_z))
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
        if self.state == "Correction":
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
        if self.drone_x is None or self.drone_y is None or self.drone_z is None or self.vx is None or self.vy is None:
            self.publish_velocity_setpoint(0.0, 0.0, 0.0)
            self.get_logger().info("Camera x, y, z or velocity x, y not available.")
            return

        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            if not self.setpoint_published:
                self.get_logger().info("[C] Publishing velocity setpoint at position x=%.2f, y=%.2f" % (self.drone_x, self.drone_y))
                self.publish_velocity_setpoint(self.vx, self.vy, 0.0)
                self.setpoint_published = False

            # Condition to switch to descent state
            if self.distance_to_desired_position(self.drone_x, self.drone_y, 0.0, 0.0) < self.error_threshold_xz:
                self.get_logger().info("[C] Horizontal Error = %.2fm" % (self.distance_to_desired_position(self.drone_x, self.drone_y, 0.0, 0.0)))
                self.state = "Descent"

        else:
            self.get_logger().info("Vehicle not in offboard mode.")


        self.drone_x = None
        self.drone_y = None
        self.drone_z = None

    def descend(self):
        """Descend the predefined height."""
        if  self.drone_z is None:
            self.publish_velocity_setpoint(0.0, 0.0, 0.0)
            self.get_logger().info("Current z, camera z, desired z or descent height not available.")
            return
        
        # Adaptive descent height
        if (self.drone_z > 2.0):
            self.descent_height = 0.5
        else:
            self.descent_height = 0.2

        # Calculate error in z
        error_z = self.drone_z - self.descent_height

  
        if not self.setpoint_published:
            self.goal_z = self.current_z + self.descent_height
            self.camera_goal_z = self.drone_z - self.descent_height
            self.setpoint_published = True
        
        error_z = self.drone_z - self.camera_goal_z

        self.vz = self.kz * error_z
        self.get_logger().info("[D] Publishing velocity setpoint at heigh: %.2f" % self.drone_z)
        self.publish_velocity_setpoint(0.0, 0.0, self.vz)
    
        # Condition to switch to landing state
        if abs(self.drone_z) < self.land_dist_th:
            self.state = "Landing"

        if abs(error_z) < self.error_threshold_z:
            # Print error information
            self.get_logger().info("[D] Vertical Error = %.2fm" % abs(error_z))
            self.state = "Correction"
            # Reset setpoint_published flag
            self.setpoint_published = False     

        self.drone_x = None
        self.drone_y = None
        self.drone_z = None 

    def distance_to_desired_position(self, x1, y1, x2, y2):
        """Calculate the Euclidean distance between two points."""
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

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