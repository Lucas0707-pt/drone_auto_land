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

        self.aruco_pose_camera_sub = self.create_subscription(
            PoseStamped, 'aruco_pose_camera', self.aruco_pose_camera_callback, 10)
        
        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        
        self.aruco_pose_local_pub = self.create_publisher(
            PoseStamped, 'aruco_pose_local', 10)
        
        self.aruco_image_sub = self.create_subscription(Image, 'aruco_image_aux', self.aruco_image_callback, 10)

        self.aruco_image_pub = self.create_publisher(Image, 'aruco_image', 10)

        self.aruco_pose_camera = {
            'timestamp': None,
            'x': None,
            'y': None,
            'z': None
        }

        self.aruco_pose_local = {
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

    def draw_frames(self, cv_image):
        # Define the size of the frame
        frame_size = 50

        # Draw the camera frame (in red)
        cv.arrowedLine(cv_image, (50, 50), (50 + frame_size, 50), (0, 0, 255), 2)
        cv.arrowedLine(cv_image, (50, 50), (50, 50 - frame_size), (0, 0, 255), 2)

        # Draw the drone frame (in green)
        cv.arrowedLine(cv_image, (150, 50), (150 + frame_size, 50), (0, 255, 0), 2)
        cv.arrowedLine(cv_image, (150, 50), (150, 50 - frame_size), (0, 255, 0), 2)

        # Draw the local frame (in blue)
        cv.arrowedLine(cv_image, (250, 50), (250 + frame_size, 50), (255, 0, 0), 2)
        cv.arrowedLine(cv_image, (250, 50), (250, 50 - frame_size), (255, 0, 0), 2)

        return cv_image

    def aruco_image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if (self.aruco_pose_camera['x'] is not None and self.aruco_pose_local['x'] is not None and self.vehicle_odometry['x'] is not None):
            aruco_pose_camera_text = f"ArUco Pose Camera: x={self.aruco_pose_camera['x']:.2f}, y={self.aruco_pose_camera['y']:.2f}, z={self.aruco_pose_camera['z']:.2f}"
            aruco_pose_local_text = f"ArUco Pose Local: x={self.aruco_pose_local['x']:.2f}, y={self.aruco_pose_local['y']:.2f}, z={self.aruco_pose_local['z']:.2f}"
            vehicle_odometry_text = f"Vehicle Odometry: x={self.vehicle_odometry['x']:.2f}, y={self.vehicle_odometry['y']:.2f}, z={self.vehicle_odometry['z']:.2f}"

            cv.putText(cv_image, aruco_pose_camera_text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv.putText(cv_image, aruco_pose_local_text, (10, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv.putText(cv_image, vehicle_odometry_text, (10, 70), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        #cv_image = self.draw_frames(cv_image)

        if self.record:
            self.out.write(cv_image)

        self.aruco_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))


    def frame_conversion(self):
        if (self.vehicle_odometry['timestamp'] is not None) and (self.aruco_pose_camera['timestamp'] is not None):
            timestamp_diff = self.vehicle_odometry['timestamp'] - self.aruco_pose_camera['timestamp']
            #self.get_logger().info('Timestamp difference: {}'.format(timestamp_diff))
            if abs(timestamp_diff) < self.acceptable_timestamp_diff:
                aruco_pose_message = PoseStamped()
                aruco_pose_drone = self.convert_cam2drone()
                self.tvec_D_L = np.array([self.vehicle_odometry['x'], self.vehicle_odometry['y'], self.vehicle_odometry['z']])
                self.rvec_D_L = self.convert_quat2rot()
                self.convert_drone2local(aruco_pose_drone)
                aruco_pose_message.pose.position.x = self.aruco_pose_local['x']
                aruco_pose_message.pose.position.y = self.aruco_pose_local['y']
                aruco_pose_message.pose.position.z = self.aruco_pose_local['z']
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
        self.aruco_pose_local['x'] = aruco_pose_local[0]
        self.aruco_pose_local['y'] = aruco_pose_local[1]
        self.aruco_pose_local['z'] = aruco_pose_local[2]

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