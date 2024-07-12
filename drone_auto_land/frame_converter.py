import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
import datetime



class FrameConverter(Node):
    def __init__(self):
        super().__init__('frame_converter')
        self.declare_parameter('record', 0)
        self.record = bool(self.get_parameter('record').get_parameter_value().integer_value)

        if self.record:
            # Define the codec and create VideoWriter object
            fourcc = cv.VideoWriter_fourcc(*'XVID')
            now = datetime.datetime.now()
            filename = f'src/drone_auto_land/videos/{now.year}_{now.month}_{now.day}_{now.hour}_{now.minute}_{now.second}.avi'
            self.out = cv.VideoWriter(filename, fourcc, 30.0, (640, 480))

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.bridge = CvBridge()

        self.pose_marker_to_camera_sub = self.create_subscription(
            PoseStamped, 'pose_marker_to_camera', self.pose_marker_to_camera_callback, 10)
        
        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        
        self.pose_drone_to_marker_pub = self.create_publisher(
            PoseStamped, 'pose_drone_to_marker', 10)
        
        self.aruco_image_sub = self.create_subscription(Image, 'aruco_image_aux', self.aruco_image_callback, 10)

        self.aruco_image_pub = self.create_publisher(Image, 'aruco_image', 10)

        self.pose_marker_to_camera = {
            'timestamp': None,
            'x': None,
            'y': None,
            'z': None,
            'qx': None,
            'qy': None,
            'qz': None,
            'qw': None
        }

        self.pose_drone_to_marker = {
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

        self.tvec_D_C = np.array([0, 0.05, -0.04])
        self.rvec_D_C = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])

        self.tvec_C_M = None
        self.rvec_C_M = None

        self.acceptable_timestamp_diff = 3 * 1e5

    def aruco_image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if (self.pose_marker_to_camera['x'] is not None and self.aruco_pose_local['x'] is not None and self.vehicle_odometry['x'] is not None):
            pose_marker_to_camera_text = f"Pose Marker to Camera: x={self.pose_marker_to_camera['x']:.2f}, y={self.pose_marker_to_camera['y']:.2f}, z={self.pose_marker_to_camera['z']:.2f}"
            pose_drone_to_marker_text = f"Pose Drone to Marker: x={self.pose_drone_to_marker['x']:.2f}, y={self.pose_drone_to_marker['y']:.2f}, z={self.pose_drone_to_marker['z']:.2f}"
            vehicle_odometry_text = f"Vehicle Odometry: x={self.vehicle_odometry['x']:.2f}, y={self.vehicle_odometry['y']:.2f}, z={self.vehicle_odometry['z']:.2f}"

            cv.putText(cv_image, pose_marker_to_camera_text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv.putText(cv_image, pose_drone_to_marker_text, (10, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv.putText(cv_image, vehicle_odometry_text, (10, 70), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        if self.record:
            self.out.write(cv_image)

        self.aruco_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))


    def convert_quat2rot(self, qx, qy, qz, qw):
        return np.array([
            [1 - 2 * qy**2 - 2 * qz**2, 2 * qx * qy - 2 * qz * qw, 2 * qx * qz + 2 * qy * qw],
            [2 * qx * qy + 2 * qz * qw, 1 - 2 * qx**2 - 2 * qz**2, 2 * qy * qz - 2 * qx * qw],
            [2 * qx * qz - 2 * qy * qw, 2 * qy * qz + 2 * qx * qw, 1 - 2 * qx**2 - 2 * qy**2]
        ])
    
    def get_transform_matrix(self, tvec, rvec):
        T = np.eye(4)
        T[:3, :3] = rvec
        T[:3, 3] = tvec
        return T

            
    def convert_drone2cam(self, point):
        T_D_C = self.get_transform_matrix(self.tvec_D_C, self.rvec_D_C)
        pose_drone_to_camera = np.dot(T_D_C, point)
        return pose_drone_to_camera
    
    def convert_camera2marker(self, point):
        rvec_M_C = self.convert_quat2rot(self.pose_marker_to_camera['qx'], self.pose_marker_to_camera['qy'], self.pose_marker_to_camera['qz'], self.pose_marker_to_camera['qw'])
        self.rvec_C_M = rvec_M_C.T
        tvec_M_C = np.array([self.pose_marker_to_camera['x'], self.pose_marker_to_camera['y'], self.pose_marker_to_camera['z']])
        self.tvec_C_M = -np.dot(self.rvec_C_M, tvec_M_C)
        T_C_M = self.get_transform_matrix(self.tvec_C_M, self.rvec_C_M)
        pose_camera_to_marker = np.dot(T_C_M, point)
        return pose_camera_to_marker




    def frame_conversion(self):
        if (self.vehicle_odometry['timestamp'] is not None) and (self.pose_marker_to_camera['timestamp'] is not None):
            timestamp_diff = self.vehicle_odometry['timestamp'] - self.pose_marker_to_camera['timestamp']

            if abs(timestamp_diff) < self.acceptable_timestamp_diff:
                pose_drone_to_marker = PoseStamped()
                pose_drone_to_drone = np.array([0, 0, 0, 1])
                pose_drone_to_camera = self.convert_drone2cam(pose_drone_to_drone)
                pose_camera_to_marker = self.convert_camera2marker(pose_drone_to_camera)

                self.pose_drone_to_marker['x'] = pose_camera_to_marker[0]
                self.pose_drone_to_marker['y'] = pose_camera_to_marker[1]
                self.pose_drone_to_marker['z'] = pose_camera_to_marker[2]

                pose_drone_to_marker.header.stamp = self.get_clock().now().to_msg()
                pose_drone_to_marker.pose.position.x = pose_camera_to_marker['x']
                pose_drone_to_marker.pose.position.y = pose_camera_to_marker['y']
                pose_drone_to_marker.pose.position.z = pose_camera_to_marker['z']
                self.pose_drone_to_marker_pub.publish(pose_drone_to_marker)

    def vehicle_odometry_callback(self, vehicle_odometry):
        self.vehicle_odometry['timestamp'] = vehicle_odometry.timestamp
        self.vehicle_odometry['x'] = vehicle_odometry.position[0]
        self.vehicle_odometry['y'] = vehicle_odometry.position[1]
        self.vehicle_odometry['z'] = vehicle_odometry.position[2]
        self.vehicle_odometry['qw'] = vehicle_odometry.q[0]
        self.vehicle_odometry['qx'] = vehicle_odometry.q[1]
        self.vehicle_odometry['qy'] = vehicle_odometry.q[2]
        self.vehicle_odometry['qz'] = vehicle_odometry.q[3]

    def pose_marker_to_camera_callback(self, pose_marker_to_camera):
        self.pose_marker_to_camera['timestamp'] = pose_marker_to_camera.header.stamp.sec * 1e6 + pose_marker_to_camera.header.stamp.nanosec * 1e-3
        self.pose_marker_to_camera['x'] = pose_marker_to_camera.pose.position.x
        self.pose_marker_to_camera['y'] = pose_marker_to_camera.pose.position.y
        self.pose_marker_to_camera['z'] = pose_marker_to_camera.pose.position.z
        self.pose_marker_to_camera['qx'] = pose_marker_to_camera.pose.orientation.x
        self.pose_marker_to_camera['qy'] = pose_marker_to_camera.pose.orientation.y
        self.pose_marker_to_camera['qz'] = pose_marker_to_camera.pose.orientation.z
        self.pose_marker_to_camera['qw'] = pose_marker_to_camera.pose.orientation.w
        self.frame_conversion()
        
def main(args=None):
    rclpy.init(args=args)
    frame_converter = FrameConverter()
    rclpy.spin(frame_converter)
    frame_converter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()