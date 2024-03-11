import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseStamped

class OffsetCalculator(Node):
    def __init__(self):
        super().__init__('offset_calculator')
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)

        self.aruco_pose_local_subscriber = self.create_subscription(
            PoseStamped, 'aruco_pose_local', self.aruco_pose_local_callback, 10)
        
        self.offset_publisher = self.create_publisher(
            PoseStamped, 'offset_correction', 10)
        
        self.current_x = None
        self.current_y = None
        self.current_z = None

        self.aruco_pose_x = None
        self.aruco_pose_y = None
        self.aruco_pose_z = None

    def calculate_offset(self):
        if self.current_x is not None and self.aruco_pose_x is not None:
            offset_x = self.aruco_pose_x - self.current_x
            offset_y = self.aruco_pose_y - self.current_y
            offset_z = self.aruco_pose_z - self.current_z
            offset = PoseStamped()
            offset.pose.position.x = offset_x
            offset.pose.position.y = offset_y
            offset.pose.position.z = offset_z
            self.offset_publisher.publish(offset)
            

        
    def vehicle_odometry_callback(self, vehicle_odometry):
        self.current_x = vehicle_odometry.position[0]
        self.current_y = vehicle_odometry.position[1]
        self.current_z = vehicle_odometry.position[2]


    def aruco_pose_local_callback(self, aruco_pose_local):
        self.aruco_pose_x = aruco_pose_local.pose.position.x
        self.aruco_pose_y = aruco_pose_local.pose.position.y
        self.aruco_pose_z = aruco_pose_local.pose.position.z
        self.calculate_offset()
        
def main(args=None):
    rclpy.init(args=args)
    offset_calculator = OffsetCalculator()
    rclpy.spin(offset_calculator)
    offset_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()