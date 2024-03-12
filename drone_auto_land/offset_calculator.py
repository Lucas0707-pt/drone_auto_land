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
        if self.aruco_pose_x is not None:
            offset = PoseStamped()
            offset_x, offset_y = self.convert_camtouav(self.aruco_pose_x, self.aruco_pose_y)
            offset.pose.position.x = offset_x
            offset.pose.position.y = offset_y
            self.offset_publisher.publish(offset)
            

    def convert_camtouav(self, x_cam, y_cam):
        x_uav = -x_cam
        y_uav = y_cam

        return x_uav, y_uav 


    def aruco_pose_local_callback(self, aruco_pose_local):
        self.aruco_pose_x = aruco_pose_local.pose.position.x/100.0
        self.aruco_pose_y = aruco_pose_local.pose.position.y/100.0
        self.aruco_pose_z = aruco_pose_local.pose.position.z/100.0
        self.calculate_offset()
        
def main(args=None):
    rclpy.init(args=args)
    offset_calculator = OffsetCalculator()
    rclpy.spin(offset_calculator)
    offset_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()