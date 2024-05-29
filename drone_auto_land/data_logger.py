import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import csv
import datetime
import os

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        self.count = 0
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
        self.aruco_pose_local_sub = self.create_subscription(
            PoseStamped, 'aruco_pose_local', self.aruco_pose_local_callback, 10)
        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.current_land_state_sub = self.create_subscription(
            String, 'current_land_state', self.current_land_state_callback, 10)

    def current_land_state_callback(self, msg):
        data = {
            'timestamp': int(datetime.datetime.now().timestamp() * 1e6),
            'state': msg.data
        }
        self.save_data_land_state('current_land_state.csv', data)
    
    def aruco_pose_camera_callback(self, msg):
        data = {
            'timestamp': int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec * 1e-3),
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }
        self.save_data('aruco_pose_camera.csv', data)

    def aruco_pose_local_callback(self, msg):
        data = {
            'timestamp': int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec * 1e-3),
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }
        self.save_data('aruco_pose_local.csv', data)

    def vehicle_odometry_callback(self, msg):
        data = {
            'timestamp': msg.timestamp,
            'x': float(msg.position[0]),
            'y': float(msg.position[1]),
            'z': float(msg.position[2])
        }
        self.save_data('vehicle_odometry.csv', data)

    def save_data(self, filename, data):
        file_path = os.path.join(self.foldername, filename)
        file_exists = os.path.isfile(file_path)

        with open(file_path, 'a', newline='') as f:
            writer = csv.writer(f)

            if not file_exists:
                writer.writerow(['timestamp', 'x', 'y', 'z'])  # Write the header

            # Write the data for the topic
            writer.writerow([data['timestamp'], data['x'], data['y'], data['z']])

    def save_data_land_state(self, filename, data):
        file_path = os.path.join(self.foldername, filename)
        file_exists = os.path.isfile(file_path)

        with open(file_path, 'a', newline='') as f:
            writer = csv.writer(f)

            if not file_exists:
                writer.writerow(['timestamp', 'state'])

            writer.writerow([data['timestamp'], data['state']])
        
        if data['state'] == 'Landed':
            exit()




def main(args=None):
    rclpy.init(args=args)
    data_logger = DataLogger()
    rclpy.spin(data_logger)
    data_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()