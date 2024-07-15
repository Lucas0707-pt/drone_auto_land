import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry, TrajectorySetpoint
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import csv
import datetime
import os

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        now = datetime.datetime.now()
        self.foldername = f'src/drone_auto_land/logs/{now.year}_{now.month}_{now.day}_{now.hour}_{now.minute}_{now.second}'
        os.makedirs(self.foldername, exist_ok=True)  # Create the directory if it doesn't exist

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.aruco_pose_camera_sub = self.create_subscription(
            PoseStamped, 'aruco_pose_camera', self.aruco_pose_camera_callback, 10)
        self.aruco_pose_drone_sub = self.create_subscription(
            PoseStamped, 'aruco_pose_drone', self.aruco_pose_drone_callback, 10)
        
        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.current_land_state_sub = self.create_subscription(
            String, 'current_land_state', self.current_land_state_callback, 10)
        self.trajectory_setpoint_sub = self.create_subscription(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.trajectory_setpoint_callback, qos_profile)
        
    def current_land_state_callback(self, msg):
        data = {
            'timestamp': int(datetime.datetime.now().timestamp() * 1e6),
            'state': msg.data
        }
        self.save_data_land_state('current_land_state.csv', data)
    
    def aruco_pose_camera_callback(self, msg):
        data = {
            'timestamp': int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec * 1e-3),
            'position_x': msg.pose.position.x,
            'position_y': msg.pose.position.y,
            'position_z': msg.pose.position.z
        }
        self.save_data_pose('aruco_pose_camera.csv', data)

    def aruco_pose_drone_callback(self, msg):
        data = {
            'timestamp': int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec * 1e-3),
            'position_x': msg.pose.position.x,
            'position_y': msg.pose.position.y,
            'position_z': msg.pose.position.z
        }
        self.save_data_pose('aruco_pose_drone.csv', data)

    def vehicle_odometry_callback(self, msg):
        data = {
            'timestamp': msg.timestamp,
            'position_x': float(msg.position[0]),
            'position_y': float(msg.position[1]),
            'position_z': float(msg.position[2]),
            'q_0': float(msg.q[0]),
            'q_1': float(msg.q[1]),
            'q_2': float(msg.q[2]),
            'q_3': float(msg.q[3]),
            'velocity_x': float(msg.velocity[0]),
            'velocity_y': float(msg.velocity[1]),
            'velocity_z': float(msg.velocity[2]),
            'angular_velocity_x': float(msg.angular_velocity[0]),
            'angular_velocity_y': float(msg.angular_velocity[1]),
            'angular_velocity_z': float(msg.angular_velocity[2]),
            'position_variance_x': float(msg.position_variance[0]),
            'position_variance_y': float(msg.position_variance[1]),
            'position_variance_z': float(msg.position_variance[2]),
            'orientation_variance_x': float(msg.orientation_variance[0]),
            'orientation_variance_y': float(msg.orientation_variance[1]),
            'orientation_variance_z': float(msg.orientation_variance[2]),
            'velocity_variance_x': float(msg.velocity_variance[0]),
            'velocity_variance_y': float(msg.velocity_variance[1]),
            'velocity_variance_z': float(msg.velocity_variance[2])
        }
        self.save_data_odometry('vehicle_odometry.csv', data)

    def trajectory_setpoint_callback(self, msg):
        data = {
            'timestamp': msg.timestamp,
            'velocity_x': float(msg.velocity[0]),
            'velocity_y': float(msg.velocity[1]),
            'velocity_z': float(msg.velocity[2]),
        }
        self.save_data_trajectory('trajectory_setpoint.csv', data)


    def save_data_pose(self, filename, data):
        file_path = os.path.join(self.foldername, filename)
        file_exists = os.path.isfile(file_path)

        with open(file_path, 'a', newline='') as f:
            writer = csv.writer(f)

            if not file_exists:
                writer.writerow(['timestamp', 'position_x', 'position_y', 'position_z'])  # Write the header

            # Write the data for the topic
            writer.writerow([data['timestamp'], data['position_x'], data['position_y'], data['position_z']])


    def save_data_odometry(self, filename, data):
        file_path = os.path.join(self.foldername, filename)
        file_exists = os.path.isfile(file_path)

        with open(file_path, 'a', newline='') as f:
            writer = csv.writer(f)

            if not file_exists:
                writer.writerow(['timestamp', 'position_x', 'position_y', 'position_z', 'q_0', 'q_1', 'q_2', 'q_3', 'velocity_x', 'velocity_y', 'velocity_z', 'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z', 'position_variance_x', 'position_variance_y', 'position_variance_z', 'orientation_variance_x', 'orientation_variance_y', 'orientation_variance_z', 'velocity_variance_x', 'velocity_variance_y', 'velocity_variance_z'])

            writer.writerow([data['timestamp'], data['position_x'], data['position_y'], data['position_z'], data['q_0'], data['q_1'], data['q_2'], data['q_3'], data['velocity_x'], data['velocity_y'], data['velocity_z'], data['angular_velocity_x'], data['angular_velocity_y'], data['angular_velocity_z'], data['position_variance_x'], data['position_variance_y'], data['position_variance_z'], data['orientation_variance_x'], data['orientation_variance_y'], data['orientation_variance_z'], data['velocity_variance_x'], data['velocity_variance_y'], data['velocity_variance_z']])

    def save_data_trajectory(self, filename, data):
        file_path = os.path.join(self.foldername, filename)
        file_exists = os.path.isfile(file_path)

        with open(file_path, 'a', newline='') as f:
            writer = csv.writer(f)

            if not file_exists:
                writer.writerow(['timestamp', 'velocity_x', 'velocity_y', 'velocity_z'])

            writer.writerow([data['timestamp'], data['velocity_x'], data['velocity_y'], data['velocity_z']])
          
    def save_data_land_state(self, filename, data):
        file_path = os.path.join(self.foldername, filename)
        file_exists = os.path.isfile(file_path)

        with open(file_path, 'a', newline='') as f:
            writer = csv.writer(f)

            if not file_exists:
                writer.writerow(['timestamp', 'state'])

            writer.writerow([data['timestamp'], data['state']])




def main(args=None):
    rclpy.init(args=args)
    data_logger = DataLogger()
    rclpy.spin(data_logger)
    data_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
