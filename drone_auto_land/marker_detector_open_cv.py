import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from geometry_msgs.msg import PoseStamped

class MarkerDetectorOpenCV(Node):
    def __init__(self):
        super().__init__('marker_detector_open_cv')

        # Create publishers and subscribers
        self.camera_image_sub = self.create_subscription(Image, 'camera', self.image_callback, 10)
        self.aruco_image_pub = self.create_publisher(Image, 'aruco_image_aux', 10)
        self.aruco_pose_camera_pub = self.create_publisher(PoseStamped, 'aruco_pose_camera', 10)

        self.bridge = CvBridge()

        self.marker_id = 29
        self.embedded_marker_id = 33
        self.marker_size = 0.295
        self.embedded_marker_size = 0.042

        # Load the camera matrix and distortion ro
        camera_matrix_file = 'src/drone_auto_land/drone_auto_land/camera_parameters/camera_matrix.txt'
        distortion_coefficients_file = 'src/drone_auto_land/drone_auto_land/camera_parameters/camera_distortion_coefficients.txt'
        self.camera_matrix = np.loadtxt(camera_matrix_file, delimiter=',')
        self.distortion_coeffs = np.loadtxt(distortion_coefficients_file, delimiter=',')

    # Publish the pose of the detected ArUco marker
    def publish_aruco_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        self.aruco_pose_camera_pub.publish(pose)

    def estimate_pose(self, corners, marker_size):
        ret = cv.aruco.estimatePoseSingleMarkers(corners, marker_size, self.camera_matrix, self.distortion_coeffs)
        tvec = ret[1][0, 0, :]
        return tvec

    # Callback function for the camera image
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_image = cv.undistort(cv_image, self.camera_matrix, self.distortion_coeffs)
        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        gray = cv.GaussianBlur(gray, (3, 3), 0)

        # Perform marker detection
        aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_7X7_1000)
        parameters = cv.aruco.DetectorParameters_create()
        #parameters.perspectiveRemoveIgnoredMarginPerCell = 0.4
        corners, ids, _ = cv.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # If at least one marker detected
        if len(corners) > 0:
            # Draw the detected markers
            cv_image = cv.aruco.drawDetectedMarkers(cv_image, corners, ids)

            # Center of the marker
            for i in range(len(corners)):
                c = corners[i][0]
                cx = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
                cy = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)
                cv.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)

            tvec = None
            #print(corners)
            match len(ids):
                case 1:
                    tvec = self.estimate_pose(corners, (self.marker_size if ids[0][0] == self.marker_id else self.embedded_marker_size))
                case 2:
                    # Check which of the corners is the big marker and estimate its pose
                    if ids[0][0] == self.marker_id:
                        tvec = self.estimate_pose([corners[0]], self.marker_size)
                    else:
                        tvec = self.estimate_pose([corners[1]], self.marker_size)
                case _:
                    self.get_logger().info("More than 2 markers detected")
            
            if tvec is not None:
                self.publish_aruco_pose(tvec[0], tvec[1], tvec[2])

        # Publish the image with the detected markers
        self.aruco_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))

def main(args=None):
    rclpy.init(args=args)
    marker_detector = MarkerDetectorOpenCV()
    rclpy.spin(marker_detector)
    marker_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()