import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import OffboardControlMode, GotoSetpoint, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus
import math
import numpy as np

class OffboardLandingController(Node):
    """Node for controlling a vehicle in offboard mode and handling landing."""

    def __init__(self) -> None:
        super().__init__('offboard_landing_controller')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.goto_setpoint_publisher = self.create_publisher( 
            GotoSetpoint, '/fmu/in/goto_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.offset_correction_subscriber = self.create_subscription(
            PoseStamped, '/offset_correction', self.offset_correction_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        self.offset_correction = PoseStamped()

        # Parameters for landing controller
        self.desired_x = 0.0
        self.desired_y = 0.0
        self.error_threshold_xy = 0.1  # Threshold for x and y error
        self.error_threshold_z = 0.1  # Threshold for z error
        self.error_land = 0.1 # Threshold for landing error
        self.descent_height = 0.2  # Height to descend in z
        self.land_dist_th = -0.3 # Height to land()
        self.goal_z = 0.0
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.offset_x = None
        self.offset_y = None
        self.state = "Correction" # Correction / Descent / Landing

        # Flag to track if setpoint has been published
        self.setpoint_published = False

        self.trajectory_setpoint_correction = None
        self.trajectory_setpoint = None

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def offset_correction_callback(self, offset_correction):
        """Callback function for vehicle_offset topic subscriber."""
        self.offset_correction = offset_correction
        self.offset_x = offset_correction.pose.position[0]
        self.offset_y = offset_correction.pose.position[1]
        
    def vehicle_odometry_callback(self, vehicle_odometry):
        """Callback function for vehicle_odometry topic subscriber."""
        self.vehicle_odometry = vehicle_odometry
        self.current_x = vehicle_odometry.position[0]
        self.current_y = vehicle_odometry.position[1]
        self.current_z = vehicle_odometry.position[2]

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def land(self):
        """Command the vehicle to land at its current altitude."""
        # current_altitude = self.current_z if self.current_z is not None else 0.0
        # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, param7=float(current_altitude))
        # self.get_logger().info('Land command sent')

        self.goto_setpoint(self.current_x, self.current_y, 0)

        if(abs(self.current_z) < self.error_land):
            self.disarm()
            #exit(0)


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

    def goto_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = GotoSetpoint()

        # Create a NumPy array with x, y, z values
        position_array = np.array([x, y, z], dtype=np.float32)

        # Assign the array to msg.position
        msg.position = position_array
        msg.heading = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.goto_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()
        self.update_desired_position()
        
        if self.state == "Correction":
            self.correct_xy_position()
        elif self.state == "Descent":
            self.descend()
        elif self.state == "Landing":
            self.land()

    def update_desired_position(self):
        # Use /offset_correction
        if(self.offset_x is not None and self.offset_y is not None):
            self.desired_x = self.current_x + self.offset_x
            self.desired_y = self.current_y + self.offset_y
        else:
            self.desired_x = self.current_x
            self.desired_y = self.current_y
            self.get_logger().info("Offset was None")

    def correct_xy_position(self):
        if self.current_x is None or self.current_y is None:
            return

        error_x = self.desired_x - self.current_x
        error_y = self.desired_y - self.current_y
        
        if self.trajectory_setpoint_correction is None:
            self.trajectory_setpoint_correction = TrajectorySetpoint()

        if not self.setpoint_published:
            # Print correction information
            self.get_logger().info("Correcting to x=%.2f, y=%.2f" % (self.desired_x, self.desired_y))

            self.trajectory_setpoint_correction.position[0] = self.desired_x
            self.trajectory_setpoint_correction.position[1] = self.desired_y
            self.trajectory_setpoint_correction.position[2] = self.current_z

            self.setpoint_published = True

        #self.trajectory_setpoint_publisher.publish(self.trajectory_setpoint_correction)
        self.goto_setpoint(self.trajectory_setpoint_correction.position[0], self.trajectory_setpoint_correction.position[1], self.trajectory_setpoint_correction.position[2])

        if abs(error_x) < self.error_threshold_xy and abs(error_y) < self.error_threshold_xy:
            # Print error information
            self.get_logger().info("Error in x=%.2f, y=%.2f" % (error_x, error_y))
            self.state = "Descent"
            # Reset setpoint_published flag
            self.setpoint_published = False

    def descend(self):
        if self.current_z is None:
            return

        error_z = self.current_z - self.descent_height
        

        if self.trajectory_setpoint is None:
            self.trajectory_setpoint = TrajectorySetpoint()

        if not self.setpoint_published:
            # Print descent information
            self.get_logger().info("Descending height=%.2f meters" % self.descent_height)

            self.goal_z = self.current_z + self.descent_height

            self.trajectory_setpoint.position[0] = self.desired_x
            self.trajectory_setpoint.position[1] = self.desired_y
            self.trajectory_setpoint.position[2] = self.goal_z
            

            self.setpoint_published = True
        
        #self.trajectory_setpoint_publisher.publish(self.trajectory_setpoint)
        self.goto_setpoint(self.trajectory_setpoint.position[0], self.trajectory_setpoint.position[1], self.trajectory_setpoint.position[2])

        error_z = self.current_z - self.goal_z

        if(self.current_z > self.land_dist_th):
            self.state = "Landing"
            #self.land()
            #self.disarm()
            #exit(0)

        if abs(error_z) < self.error_threshold_z:
            # Print error information
            self.get_logger().info("Error in z=%.2f" % error_z)
            self.state = "Correction"
            # Reset setpoint_published flag
            self.setpoint_published = False

    

def main(args=None) -> None:
    print('Starting offboard landing controller node...')
    rclpy.init(args=args)
    offboard_landing_controller = OffboardLandingController()

    # Set desired landing position
    #offboard_landing_controller.desired_x = 0
    #offboard_landing_controller.desired_y = 0

    rclpy.spin(offboard_landing_controller)
    offboard_landing_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
