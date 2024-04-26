import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import json
import time
import datetime

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        now = datetime.datetime.now()
        self.filename = f'src/drone_auto_land/logs/{now.year}_{now.month}_{now.day}_{now.hour}_{now.minute}_{now.second}.json'

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

        self.data = {
            'aruco_pose_camera': [],
            'aruco_pose_local': [],
            'vehicle_odometry': []
        }

    def aruco_pose_camera_callback(self, msg):
        self.data['aruco_pose_camera'].append({
            'timestamp': time.time(),
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        })
        self.save_data()

    def aruco_pose_local_callback(self, msg):
        self.data['aruco_pose_local'].append({
            'timestamp': time.time(),
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        })
        self.save_data()

    def vehicle_odometry_callback(self, msg):
        self.data['vehicle_odometry'].append({
            'timestamp': time.time(),
            'x': float(msg.position[0]), # Convert numpy float32 to Python float
            'y': float(msg.position[1]),
            'z': float(msg.position[2])
        })
        self.save_data()

    def save_data(self):
        with open(self.filename, 'w') as f:
            json.dump(self.data, f)

def main(args=None):
    rclpy.init(args=args)
    data_logger = DataLogger()
    rclpy.spin(data_logger)
    data_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
