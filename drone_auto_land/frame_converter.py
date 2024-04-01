import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseStamped
import math

class FrameConverter(Node):
    def __init__(self):
        super().__init__('frame_converter')

        self.camera_offset = 0.12
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

        self.acceptable_timestamp_diff = 3 * 1e5

    def frame_conversion(self):
        if (self.vehicle_odometry['timestamp'] is not None) and (self.aruco_pose_camera['timestamp'] is not None):
            timestamp_diff = self.vehicle_odometry['timestamp'] - self.aruco_pose_camera['timestamp']
            #self.get_logger().info('Timestamp difference: {}'.format(timestamp_diff))
            if abs(timestamp_diff) < self.acceptable_timestamp_diff:
                print("timestamp_diff:", timestamp_diff)
                aruco_pose_local = PoseStamped()
                aruco_pose_uav_x, aruco_pose_uav_y = self.convert_cam2uav()
                yaw = self.convertquat2yaw()
                aruco_pose_local_x, aruco_pose_local_y = self.convert_uav2local(yaw, aruco_pose_uav_x, aruco_pose_uav_y)
                aruco_pose_local.pose.position.x = aruco_pose_local_x
                aruco_pose_local.pose.position.y = aruco_pose_local_y
                self.aruco_pose_local_pub.publish(aruco_pose_local)


    def convertquat2yaw(self):
        t3 = 2.0 * (self.vehicle_odometry['qw'] * self.vehicle_odometry['qz'] + self.vehicle_odometry['qx'] * self.vehicle_odometry['qy'])
        t4 = 1.0 - 2.0 * (self.vehicle_odometry['qy'] * self.vehicle_odometry['qy'] + self.vehicle_odometry['qz'] * self.vehicle_odometry['qz'])
        yaw = math.atan2(t3, t4)
        return yaw
            
    def convert_cam2uav(self):
        x_uav = -self.aruco_pose_camera['y'] + self.camera_offset
        y_uav = self.aruco_pose_camera['x']

        return x_uav, y_uav 
    
    def convert_uav2local(self, yaw, x_uav, y_uav):
        x_local = (math.cos(yaw) * x_uav - math.sin(yaw) * y_uav) + self.vehicle_odometry['x']
        y_local = (math.sin(yaw) * x_uav + math.cos(yaw) * y_uav) + self.vehicle_odometry['y']
        return x_local, y_local

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