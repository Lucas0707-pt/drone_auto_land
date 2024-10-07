import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry, TrajectorySetpoint
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
            self.out = cv.VideoWriter(filename, fourcc, 5.0, (640, 480))

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.bridge = CvBridge()

        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        
        self.trajectory_setpoint_sub = self.create_subscription(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.trajectory_setpoint_callback, qos_profile)

        self.aruco_pose_camera_sub = self.create_subscription(
            PoseStamped, 'aruco_pose_camera', self.aruco_pose_camera_callback, 10)
        
        self.aruco_pose_drone_pub = self.create_publisher(PoseStamped, 'aruco_pose_drone', 10)
        
        self.aruco_image_sub = self.create_subscription(Image, 'aruco_image_aux', self.aruco_image_callback, 10)

        self.aruco_image_pub = self.create_publisher(Image, 'aruco_image', 10)

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
            'z': None
        }

        self.velocity_setpoint = {
            'timestamp': None,
            'vx': None,
            'vy': None,
            'vz': None
        }

        self.tvec_C_D = np.array([0.06, 0, 0.04])
        self.rvec_C_D = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])

    def aruco_image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if (self.aruco_pose_camera['x'] is not None and self.vehicle_odometry['x'] is not None):
            aruco_pose_camera_text = f"ArUco Pose Camera: x={self.aruco_pose_camera['x']:.2f}, y={self.aruco_pose_camera['y']:.2f}, z={self.aruco_pose_camera['z']:.2f}"
            vehicle_odometry_text = f"Drone Position: x={self.vehicle_odometry['x']:.2f}, y={self.vehicle_odometry['y']:.2f}, z={self.vehicle_odometry['z']:.2f}"
            cv.putText(cv_image, aruco_pose_camera_text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv.putText(cv_image, vehicle_odometry_text, (10, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            if (self.velocity_setpoint['vx'] is not None):
                velocity_setpoint_text = f"Velocity Setpoint: vx={self.velocity_setpoint['vx']:.2f}, vy={self.velocity_setpoint['vy']:.2f}, vz={self.velocity_setpoint['vz']:.2f}"
                cv.putText(cv_image, velocity_setpoint_text, (10, 70), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        if self.record:
            self.out.write(cv_image)

        self.aruco_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))

    # Publish the pose of the detected ArUco marker
    def publish_aruco_pose_drone(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        self.aruco_pose_drone_pub.publish(pose)

    def frame_conversion(self):
        aruco_pose_drone = self.convert_cam2drone()
        self.publish_aruco_pose_drone(aruco_pose_drone[0], aruco_pose_drone[1], aruco_pose_drone[2])

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

    def aruco_pose_camera_callback(self, aruco_pose_camera):
        self.aruco_pose_camera['timestamp'] = aruco_pose_camera.header.stamp.sec * 1e6 + aruco_pose_camera.header.stamp.nanosec * 1e-3
        self.aruco_pose_camera['x'] = aruco_pose_camera.pose.position.x
        self.aruco_pose_camera['y'] = aruco_pose_camera.pose.position.y
        self.aruco_pose_camera['z'] = aruco_pose_camera.pose.position.z
        self.frame_conversion()

    def vehicle_odometry_callback(self, vehicle_odometry):
        self.vehicle_odometry['timestamp'] = vehicle_odometry.timestamp
        self.vehicle_odometry['x'] = vehicle_odometry.position[0]
        self.vehicle_odometry['y'] = vehicle_odometry.position[1]
        self.vehicle_odometry['z'] = vehicle_odometry.position[2]
        self.vehicle_odometry['qw'] = vehicle_odometry.q[0]
        self.vehicle_odometry['qx'] = vehicle_odometry.q[1]
        self.vehicle_odometry['qy'] = vehicle_odometry.q[2]
        self.vehicle_odometry['qz'] = vehicle_odometry.q[3]

    def trajectory_setpoint_callback(self, msg):
        self.velocity_setpoint['timestamp'] = msg.timestamp
        self.velocity_setpoint['vx'] = msg.velocity[0]
        self.velocity_setpoint['vy'] = msg.velocity[1]
        self.velocity_setpoint['vz'] = msg.velocity[2]
        
def main(args=None):
    rclpy.init(args=args)
    frame_converter = FrameConverter()
    rclpy.spin(frame_converter)
    frame_converter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()