import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus, VehicleTorqueSetpoint, VehicleThrustSetpoint
import numpy as np
from scipy.linalg import solve_continuous_are

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

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        self.vehicle_torque_setpoint_publisher = self.create_publisher(
            VehicleTorqueSetpoint, '/fmu/in/vehicle_torque_setpoint', qos_profile)
        
        self.vehicle_thrust_setpoint_publisher = self.create_publisher(
            VehicleThrustSetpoint, '/fmu/in/vehicle_thrust_setpoint', qos_profile)

        # Create subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        
        self.aruco_pose_local_subscriber = self.create_subscription(
            PoseStamped, '/aruco_pose_local', self.aruco_pose_local_callback, 10)
        
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        
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
        self.land_dist_th = 0.2 # Height to land()
        self.goal_z = 0.0
        self.error_threshold_z = 0.05  # Threshold for z error
        self.error_threshold_xz = 0.1 # Threshold for x and z error
        
        self.state = "Correction" # Correction / Descent / Landing

        # Initialize navigation state
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX

        # Flag to track if setpoint has been published
        self.setpoint_published = False

        # Flag to track if offboard mode has been started
        self.offboard_started = False

        # LQR variables
        self.K = None
        self.A = None
        self.B = None
        self.C = None
        self.D = None
        self.current_state = np.zeros(12)
        self.desired_state = np.zeros(12)

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.02, self.timer_callback)

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state

    def vehicle_odometry_callback(self, msg):
        """Callback function for vehicle_odometry topic subscriber."""
        self.vehicle_odometry = msg

        # Extract position and velocity data
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        self.current_z = msg.position[2]

        position = msg.position
        velocity = msg.velocity

        # Extract roll, pitch, and yaw angles from the quaternion
        q = msg.q
        q0, q1, q2, q3 = q[0], q[1], q[2], q[3]

        # Roll (phi) angle
        phi = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1**2 + q2**2))

        # Pitch (theta) angle
        theta = np.arcsin(2 * (q0 * q2 - q3 * q1))

        # Yaw (psi) angle
        psi = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2**2 + q3**2))

        # Extract angular velocity data
        angular_velocity = msg.angular_velocity

        # Update the current state
        self.current_state = [
            position[0], position[1], position[2],
            velocity[0], velocity[1], velocity[2],
            phi, theta, psi,
            angular_velocity[0], angular_velocity[1], angular_velocity[2]
        ]

    def aruco_pose_local_callback(self, aruco_pose_local):
        """Callback function for aruco_pose_local topic subscriber."""
        self.aruco_pose_local = aruco_pose_local
        self.desired_x = aruco_pose_local.pose.position.x
        self.desired_y = aruco_pose_local.pose.position.y
        self.desired_z = aruco_pose_local.pose.position.z

        self.desired_state = np.array([self.desired_x, self.desired_y, self.current_z, self.current_state[3], self.current_state[4], self.current_state[5], self.current_state[6], self.current_state[7], self.current_state[8], self.current_state[9], self.current_state[10], self.current_state[11]])

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
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
            self._logger.info('Landing at the altitude of %.2fm' % abs(self.current_z - self.desired_z))
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, param7=float(abs(self.current_z - self.desired_z)))
            self.get_logger().info('Land command sent')
            self.land_command_sent = True
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

    def publish_control_commands(self):
        """Publish control commands using VehicleTorqueSetpoint and VehicleThrustSetpoint messages."""
        u = -self.K.dot(self.current_state - self.desired_state)
        self.get_logger().info("Control Input = %s" % u)
        
        # Define the system state model
        x_dot = self.A.dot(self.current_state) + self.B.dot(u)
        y = self.C.dot(self.current_state) + self.D.dot(u)

        self.get_logger().info("x_dot = %s" % x_dot)
        self.get_logger().info("y = %s" % y)

        # Create and populate the VehicleTorqueSetpoint message
        msg_torque = VehicleTorqueSetpoint()
        msg_torque.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg_torque.xyz = [u[1], u[2], u[3]]

        msg_thrust = VehicleThrustSetpoint()
        msg_thrust.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg_thrust.xyz = [0., 0., u[0]] 

        msg = TrajectorySetpoint()
        msg.position = [y[0], y[1], ]
        msg.velocity = [x_dot[0], x_dot[1], x_dot[2]]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        
        #self.get_logger().info("Torque = %.2f, %.2f, %.2f" % (u[1], u[2], u[3]))
        #self.get_logger().info("Thrust = 0.0, 0.0, %.2f" % u[0])
        
        # Publish the control commands
        # self.vehicle_torque_setpoint_publisher.publish(msg_torque)
        # self.vehicle_thrust_setpoint_publisher.publish(msg_thrust)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        # Publish offboard control heartbeat signal
        if (self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.land_command_sent == False):
            self.engage_offboard_mode()
            self.offboard_started = True
            
        self.publish_offboard_control_heartbeat_signal()

        if self.K is None:
            self.get_logger().info("No K matrix available.")
            exit(0)
        
        # State machine
        if self.state == "Correction" and self.desired_x != 0.0 and self.desired_x != 0.0:
            self.correct_xy_position()
        elif self.state == "Descent":
            self.descend()
        elif self.state == "Landing" and not self.land_command_sent:
            self.land()

    def perform_lqr(self):
        """Perform LQR control."""
        # System parameters
        m = 2.0  # Mass (kg)
        Jx = 0.02166666666666667  # Moment of inertia around x-axis (kg*m^2)
        Jy = 0.02166666666666667  # Moment of inertia around y-axis (kg*m^2)
        Jz = 0.04000000000000001  # Moment of inertia around z-axis (kg*m^2)
        g = 9.81  # Acceleration due to gravity (m/s^2)

        # Define system matrices
        self.A = np.array([[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, g, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, g, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, g, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        
        self.B = np.array([[0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [1 / m, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 1 / Jx, 0, 0],
                    [0, 0, 1 / Jy, 0],
                    [0, 0, 0, 1 / Jz]])

        # Define weighting matrices
        Q = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])

        R = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

        # Solve the continuous-time algebraic Riccati equation (CARE) to obtain the optimal control gain
        P = solve_continuous_are(self.A, self.B, Q, R)

        # Compute the optimal control gain
        self.K = np.linalg.inv(R).dot(self.B.T).dot(P)

        # Matrices C and D
        self.C = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]])
        
        self.D = np.array([[0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [0, 0, 0, 0]])
    
    def correct_xy_position(self):
        """Correct the position of the drone in the horizontal plane."""
        if self.current_x is None or self.current_y is None or self.desired_x is None or self.desired_y is None or self.current_z is None or self.desired_z is None:
            self.get_logger().info("Current x, y, z or desired x, y, z not available.")
            return

        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            self.publish_control_commands()

            if not self.setpoint_published:
                self.get_logger().info("Current position x=%.2fm, y=%.2fm" % (self.current_x, self.current_y))
                self.get_logger().info("Correcting to x=%.2fm, y=%.2fm" % (self.desired_x, self.desired_y))

                self.get_logger().info("Current state = %s" % self.current_state)
                self.get_logger().info("Desired state = %s" % self.desired_state)
                
            #    self.publish_control_commands()
                
            #     # Generate linear trajectory for correction
            #     waypoints = self.generate_linear_trajectory(self.current_x, self.current_y, self.desired_x, self.desired_y, self.current_z, self.current_z, num_points=2)
                
            #     # Publish trajectory setpoints
            #     for waypoint in waypoints:
            #         self.publish_trajectory_setpoint(waypoint[0], waypoint[1], waypoint[2])
                    
                self.setpoint_published = True

            # Condition to switch to descent state
            if self.distance_to_desired_position(self.current_x, self.current_y, self.desired_x, self.desired_y) < self.error_threshold_xz:
                self.get_logger().info("Horizontal Error = %.2fm" % (self.distance_to_desired_position(self.current_x, self.current_y, self.desired_x, self.desired_y)))
                self.state = "Descent"
                self.setpoint_published = False

        else:
            self.get_logger().info("Vehicle not in offboard mode.")

    def descend(self):
        """Descend the predefined height."""
        if self.current_z is None or self.descent_height is None or self.desired_z is None:
            self.get_logger().info("Current z, desired z or descent height not available.")
            return
        
        # Calculate error in z
        error_z = self.current_z - self.descent_height

        self.publish_control_commands() 

        if not self.setpoint_published:
            self.goal_z = self.current_z + self.descent_height

        #    self.publish_control_commands()

            self.get_logger().info("Current position z=%.2f" % (self.current_z))
            self.get_logger().info("Descending to z=%.2f" % (self.goal_z))      

            self.get_logger().info("Current state = %s" % self.current_state)
            self.get_logger().info("Desired state = %s" % self.desired_state)             
            
        #     # Generate  z = start_z + t * (end_z - start_z)te linear trajectory for descent
        #     waypoints = self.generate_linear_trajectory(self.current_x, self.current_y, self.desired_x, self.desired_y, self.current_z, self.current_z + self.descent_height, num_points=2)
            
        #     # Publish trajectory setpoints
        #     for waypoint in waypoints:
        #         self.publish_trajectory_setpoint(waypoint[0], waypoint[1], waypoint[2])
            self.setpoint_published = True
        
        error_z = self.current_z - self.goal_z

        # Condition to switch to landing state
        if abs(self.current_z - self.desired_z) < self.land_dist_th:
            self.state = "Landing"

        if abs(error_z) < self.error_threshold_z:
            # Print error information
            self.get_logger().info("Vertical Error = %.2fm" % abs(error_z))
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


def main(args=None) -> None:
    print('Starting offboard landing controller node...')
    rclpy.init(args=args)
    offboard_landing_controller = OffboardLandingController()
    offboard_landing_controller.perform_lqr()
    print("LQR control initialized K = %s" % offboard_landing_controller.K)
    rclpy.spin(offboard_landing_controller)
    offboard_landing_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()