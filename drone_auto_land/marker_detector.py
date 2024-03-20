import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import threading
from geometry_msgs.msg import PoseStamped
import time

class MarkerDetector(Node):
    def __init__(self):
        super().__init__('marker_detector')
        self.declare_parameter('record', 0)
        self.record = bool(self.get_parameter('record').get_parameter_value().integer_value)

        self.camera_image_sub = self.create_subscription(Image, 'camera', self.image_callback, 10)
        self.aruco_image_pub = self.create_publisher(Image, 'aruco_image', 10)
        self.aruco_pose_camera_pub = self.create_publisher(PoseStamped, 'aruco_pose_camera', 10)
        self.bridge = CvBridge()
        self.font = cv.FONT_HERSHEY_PLAIN
        self.marker_size = 0.2

        # Load the camera matrix and distortion coefficients
        camera_matrix_file = 'src/drone_auto_land/drone_auto_land/camera_parameters/camera_matrix.txt'
        distortion_coefficients_file = 'src/drone_auto_land/drone_auto_land/camera_parameters/camera_distortion_coefficients.txt'
        self.camera_matrix = np.loadtxt(camera_matrix_file, delimiter=',')
        self.distortion_coeffs = np.loadtxt(distortion_coefficients_file, delimiter=',')

        if self.record:
            # Define the codec and create VideoWriter object
            fourcc = cv.VideoWriter_fourcc(*'XVID')

            curr_time = time.strftime("%d-%m-%Y_%H-%M-%S")
            filename = 'src/drone_auto_land/drone_auto_land/videos/' + curr_time
            self.out = cv.VideoWriter(filename + '.avi', fourcc, 20.0, (640, 480))

    def publish_aruco_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        self.aruco_pose_camera_pub.publish(pose)

    def image_callback(self, msg):

        # Convert the ROS Image message to a CV Image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Undistort the image
        cv_image = cv.undistort(cv_image, self.camera_matrix, self.distortion_coeffs)
        
        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        gray = cv.GaussianBlur(gray, (3, 3), 0)

        # Perform marker detection
        aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
        parameters = cv.aruco.DetectorParameters_create()
        corners, ids, _ = cv.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # If at least one marker detected
        if len(corners) > 0:
            # Draw detected markers on the image
            cv_image = cv.aruco.drawDetectedMarkers(cv_image, corners, ids)

            #center of the marker
            for i in range(len(corners)):
                c = corners[i][0]
                cx = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
                cy = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)
                cv.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)

            # Estimate pose of each marker and return the values rvec and tvec
            ret = cv.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.distortion_coeffs)
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            self.publish_aruco_pose(tvec[0], tvec[1], tvec[2])

            #cv.aruco.drawAxis(cv_image, self.camera_matrix, self.distortion_coeffs, rvec, tvec, 10)

        # Publish the image with the detected markers
        self.aruco_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))

        if self.record:
            self.out.write(cv_image)
        
def main(args=None):
    rclpy.init(args=args)
    marker_detector = MarkerDetector()
    rclpy.spin(marker_detector)
    marker_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()