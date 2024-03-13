import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseStamped
import math

class FrameConverter(Node):
    def __init__(self):
        super().__init__('offset_calculator')

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

        self.current_x = None
        self.current_y = None
        self.current_z = None

        self.aruco_pose_camera_x = None
        self.aruco_pose_camera_y = None
        self.aruco_pose_camera_z = None

        self.qw = None
        self.qx = None
        self.qy = None
        self.qz = None

    def frame_conversion(self):
        if (self.current_x is not None):
            aruco_pose_local = PoseStamped()
            aruco_pose_uav_x, aruco_pose_uav_y = self.convert_cam2uav(self.aruco_pose_camera_x, self.aruco_pose_camera_y)
            yaw = self.convertquat2yaw(self.qw, self.qx, self.qy, self.qz)
            aruco_pose_local_x, aruco_pose_local_y = self.convert_uav2local(yaw, aruco_pose_uav_x, aruco_pose_uav_y)
            aruco_pose_local.pose.position.x = aruco_pose_local_x
            aruco_pose_local.pose.position.y = aruco_pose_local_y
            self.aruco_pose_local_pub.publish(aruco_pose_local)


    def convertquat2yaw(self, qw, qx, qy, qz):
        t3 = +2.0 * (qw * qz + qx * qy)
        t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(t3, t4)
        return yaw
            
    def convert_cam2uav(self, x_cam, y_cam):
        x_uav = -y_cam + self.camera_offset
        y_uav = x_cam

        return x_uav, y_uav 
    
    def convert_uav2local(self, yaw, x_uav, y_uav):
        x_local = (math.cos(yaw) * x_uav - math.sin(yaw) * y_uav) + self.current_x
        y_local = (math.sin(yaw) * x_uav + math.cos(yaw) * y_uav) + self.current_y
        return x_local, y_local

    def vehicle_odometry_callback(self, vehicle_odometry):
        """Callback function for vehicle_odometry topic subscriber."""
        self.vehicle_odometry = vehicle_odometry
        self.current_x = vehicle_odometry.position[0]
        self.current_y = vehicle_odometry.position[1]
        self.current_z = vehicle_odometry.position[2]
        self.qw = vehicle_odometry.q[0]
        self.qx = vehicle_odometry.q[1]
        self.qy = vehicle_odometry.q[2]
        self.qz = vehicle_odometry.q[3]

    def aruco_pose_camera_callback(self, aruco_pose_camera):
        self.aruco_pose_camera_x = aruco_pose_camera.pose.position.x
        self.aruco_pose_camera_y = aruco_pose_camera.pose.position.y
        self.aruco_pose_camera_z = aruco_pose_camera.pose.position.z
        self.frame_conversion()
        
def main(args=None):
    rclpy.init(args=args)
    offset_calculator = FrameConverter()
    rclpy.spin(offset_calculator)
    offset_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()