import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseStamped
import numpy as np

class FrameConverter(Node):
    def __init__(self):
        super().__init__('frame_converter')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.aruco_pose_camera_sub = self.create_subscription(
            PoseStamped, 'aruco_pose_camera', self.aruco_pose_camera_callback, 10)
        
        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        
        self.aruco_pose_local_pub = self.create_publisher(
            PoseStamped, 'aruco_pose_local', 10)

        self.aruco_pose_camera = {
            'timestamp': None,
            'x': None,
            'y': None,
            'z': None
        }

        self.vehicle_odometry = {
            'timestamp': None,
            'x': None,
            'y': None,
            'z': None,
            'qw': None,
            'qx': None,
            'qy': None,
            'qz': None
        }

        self.tvec_C_D = np.array([0.05, 0, 0.04])
        self.rvec_C_D = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])

        
        self.tvec_D_L = None
        self.rvec_D_L = None


        self.acceptable_timestamp_diff = 3 * 1e5

    def frame_conversion(self):
        if (self.vehicle_odometry['timestamp'] is not None) and (self.aruco_pose_camera['timestamp'] is not None):
            timestamp_diff = self.vehicle_odometry['timestamp'] - self.aruco_pose_camera['timestamp']
            #self.get_logger().info('Timestamp difference: {}'.format(timestamp_diff))
            if abs(timestamp_diff) < self.acceptable_timestamp_diff:
                aruco_pose_message = PoseStamped()
                aruco_pose_drone = self.convert_cam2drone()
                self.tvec_D_L = np.array([self.vehicle_odometry['x'], self.vehicle_odometry['y'], self.vehicle_odometry['z']])
                self.rvec_D_L = self.convert_quat2rot()
                aruco_pose_local = self.convert_drone2local(aruco_pose_drone)
                aruco_pose_message.pose.position.x = aruco_pose_local[0]
                aruco_pose_message.pose.position.y = aruco_pose_local[1]
                aruco_pose_message.pose.position.z = aruco_pose_local[2]
                self.aruco_pose_local_pub.publish(aruco_pose_message)


    def convert_quat2rot(self):
        return np.array([[1 - 2 * self.vehicle_odometry['qy']**2 - 2 * self.vehicle_odometry['qz']**2, 2 * self.vehicle_odometry['qx'] * self.vehicle_odometry['qy'] - 2 * self.vehicle_odometry['qz'] * self.vehicle_odometry['qw'], 2 * self.vehicle_odometry['qx'] * self.vehicle_odometry['qz'] + 2 * self.vehicle_odometry['qy'] * self.vehicle_odometry['qw']],
                        [2 * self.vehicle_odometry['qx'] * self.vehicle_odometry['qy'] + 2 * self.vehicle_odometry['qz'] * self.vehicle_odometry['qw'], 1 - 2 * self.vehicle_odometry['qx']**2 - 2 * self.vehicle_odometry['qz']**2, 2 * self.vehicle_odometry['qy'] * self.vehicle_odometry['qz'] - 2 * self.vehicle_odometry['qx'] * self.vehicle_odometry['qw']],
                        [2 * self.vehicle_odometry['qx'] * self.vehicle_odometry['qz'] - 2 * self.vehicle_odometry['qy'] * self.vehicle_odometry['qw'], 2 * self.vehicle_odometry['qy'] * self.vehicle_odometry['qz'] + 2 * self.vehicle_odometry['qx'] * self.vehicle_odometry['qw'], 1 - 2 * self.vehicle_odometry['qx']**2 - 2 * self.vehicle_odometry['qy']**2]])
    
    def get_transform_matrix(self, tvec, rvec):
        T = np.eye(4)
        T[:3, :3] = rvec
        T[:3, 3] = tvec
        return T

            
    def convert_cam2drone(self):
        T_C_D = self.get_transform_matrix(self.tvec_C_D, self.rvec_C_D)
        aruco_pose_camera = np.array([self.aruco_pose_camera['x'], self.aruco_pose_camera['y'], self.aruco_pose_camera['z'], 1])
        aruco_pose_drone = np.dot(T_C_D, aruco_pose_camera)
        return aruco_pose_drone
        

    def convert_drone2local(self, aruco_pose_drone):
        T_D_L = self.get_transform_matrix(self.tvec_D_L, self.rvec_D_L)
        aruco_pose_local = np.dot(T_D_L, aruco_pose_drone)
        return aruco_pose_local

    def vehicle_odometry_callback(self, vehicle_odometry):
        self.vehicle_odometry['timestamp'] = vehicle_odometry.timestamp
        self.vehicle_odometry['x'] = vehicle_odometry.position[0]
        self.vehicle_odometry['y'] = vehicle_odometry.position[1]
        self.vehicle_odometry['z'] = vehicle_odometry.position[2]
        self.vehicle_odometry['qw'] = vehicle_odometry.q[0]
        self.vehicle_odometry['qx'] = vehicle_odometry.q[1]
        self.vehicle_odometry['qy'] = vehicle_odometry.q[2]
        self.vehicle_odometry['qz'] = vehicle_odometry.q[3]

    def aruco_pose_camera_callback(self, aruco_pose_camera):
        self.aruco_pose_camera['timestamp'] = aruco_pose_camera.header.stamp.sec * 1e6 + aruco_pose_camera.header.stamp.nanosec * 1e-3
        self.aruco_pose_camera['x'] = aruco_pose_camera.pose.position.x
        self.aruco_pose_camera['y'] = aruco_pose_camera.pose.position.y
        self.aruco_pose_camera['z'] = aruco_pose_camera.pose.position.z
        self.frame_conversion()
        
def main(args=None):
    rclpy.init(args=args)
    frame_converter = FrameConverter()
    rclpy.spin(frame_converter)
    frame_converter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()