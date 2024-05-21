import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from geometry_msgs.msg import PoseStamped

class MarkerDetector(Node):
    def __init__(self):
        super().__init__('marker_detector')

        #create publishers and subscribers
        self.camera_image_sub = self.create_subscription(Image, 'camera', self.image_callback, 10)

        self.aruco_image_pub = self.create_publisher(Image, 'aruco_image_aux', 10)
        self.aruco_pose_camera_pub = self.create_publisher(PoseStamped, 'aruco_pose_camera', 10)

        self.bridge = CvBridge()

        self.marker_id = 29
        self.embedded_marker_id = 33
        self.marker_bits = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0],
                                        [0, 1, 1, 0, 0, 1, 0, 1, 0],
                                        [0, 1, 1, 0, 1, 1, 0, 1, 0],
                                        [0, 0, 1, 1, 1, 0, 1, 1, 0],
                                        [0, 1, 0, 1, 1, 1, 0, 0, 0],
                                        [0, 1, 0, 1, 1, 1, 1, 0, 0],
                                        [0, 1, 1, 0, 1, 1, 0, 0, 0],
                                        [0, 0, 1, 1, 0, 0, 0, 0, 0],
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0]],dtype=np.uint8)

        self.marker_size = 0.291
        self.embedded_marker_size = 0.033

        # Load the camera matrix and distortion coefficients
        camera_matrix_file = 'camera_parameters/camera_matrix.txt'
        distortion_coefficients_file = 'camera_parameters/camera_distortion_coefficients.txt'
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

    def image_binarization(self, img):
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        threshold_img = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 11, 2)
        return threshold_img
    
    def detect_corners(self, img, min_area, max_area):
        contours, _ = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        
        group_of_corners = []
        for contour in contours:
            area = cv.contourArea(contour)
            precision = 0.04
            epsilon = precision * cv.arcLength(contour, True)
            approx = cv.approxPolyDP(contour, epsilon, True)

            if (min_area < area < max_area and len(approx) == 4):
                #approx = cv.convexHull(approx)
                approx = approx.reshape((-1, 2))
                group_of_corners.append(approx)
                #self.get_logger().info(f"Area: {area}")

        return group_of_corners

        #self.get_logger().info(f"Number of corners detected: {len(group_of_corners[0])}")
    
    def verify_potential_marker(self, img, corners, markerBorderBits=1, minOtsuStdDev=5.0, perspectiveRemovePixelPerCell=4, perspectiveRemoveIgnoredMarginPerCell=0.13):
        
        dst_pts = np.array([[0, 0], [299, 0], [299, 299], [0, 299]], dtype=np.float32)

        corners = corners.astype(np.float32)
        M = cv.getPerspectiveTransform(corners, dst_pts)
        warped = cv.warpPerspective(img, M, (300, 300))
        
        gray = cv.cvtColor(warped, cv.COLOR_BGR2GRAY)

        #Apply otsu thresholding with gaussian filtering
        gray = cv.GaussianBlur(gray, (3, 3), 0)
        _, threshold_image = cv.threshold(gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)

        # Perform marker detection
        cell_size = threshold_image.shape[0] // 9
        cell_margin = int(cell_size * perspectiveRemoveIgnoredMarginPerCell)
        bits = np.zeros((9,9), dtype=np.uint8)

        for y in range(9):
            for x in range(9):
                cell = threshold_image[y*cell_size:(y+1)*cell_size, x*cell_size:(x+1)*cell_size]
                cell = cell[cell_margin:cell_size-cell_margin, cell_margin:cell_size-cell_margin]
                if np.mean(cell) > 127:
                    bits[y,x] = 1


        bits_image = cv.cvtColor(bits.astype(np.uint8)*255, cv.COLOR_GRAY2BGR)
        return bits_image
        

    
    def estimate_pose(self, corners, marker_size):
        ret = cv.aruco.estimatePoseSingleMarkers(corners, marker_size, self.camera_matrix, self.distortion_coeffs)
        _, tvec = ret[0][0,0,:], ret[1][0,0,:]
        return tvec



    # Callback function for the camera image
    def image_callback(self, msg):
        # save the image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        undistorted_img = cv.undistort(cv_image, self.camera_matrix, self.distortion_coeffs)
        cv_image_bin = self.image_binarization(undistorted_img)
        group_of_corners = self.detect_corners(cv_image_bin, 1000, 10000)

        if len(group_of_corners) == 0:
            return
        
        cv_image = self.verify_potential_marker(undistorted_img, group_of_corners[0])
        #components = self.connected_components(cv_image_bin)
        #cv_image = self.connected_components(cv_image_bin)
        #contours, _ = cv.findContours(cv_image_bin, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        #draw the contours
        #cv_image = cv.drawContours(cv_image, contours, -1, (0, 255, 0), 3)

        """
        cv_image = cv.undistort(cv_image, self.camera_matrix, self.distortion_coeffs)
        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        _, treshold_img = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 11, 2)
        contours, _ = cv.findContours(treshold_img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        #Apply otsu thresholding with gaussian filtering
        gray = cv.GaussianBlur(gray, (3, 3), 0)
        _, threshold_image = cv.threshold(gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)


        # Perform marker detection
        aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_7X7_1000)
        parameters = cv.aruco.DetectorParameters_create()
        corners, ids, _ = cv.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # If at least one marker detected
        if len(corners) > 0:

            #Draw the detected markers
            cv_image = cv.aruco.drawDetectedMarkers(cv_image, corners, ids)

            #center of the marker
            for i in range(len(corners)):
                c = corners[i][0]
                cx = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
                cy = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)
                cv.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)

            tvec = None

            match len(ids):
                case 1:
                    tvec = self.estimate_pose(corners, (self.marker_size if ids[0][0] == self.marker_id else self.embedded_marker_size))
                case 2:
                    # check which of the corners is the big marker and estimate its pose
                    if ids[0][0] == self.marker_id:
                        tvec = self.estimate_pose([corners[0]], self.marker_size)
                    else:
                        tvec = self.estimate_pose([corners[1]], self.marker_size)
                case _:
                    self.get_logger().info("More than 2 markers detected")

            self.publish_aruco_pose(tvec[0], tvec[1], tvec[2])

            aruco_pose_camera_text = f"ArUco Pose Camera: x={tvec[0]:.2f}, y={tvec[1]:.2f}, z={tvec[2]:.2f}"
            cv.putText(cv_image, aruco_pose_camera_text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)


        # Publish the image with the detected markers
        """
        self.aruco_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8'))

        
def main(args=None):
    rclpy.init(args=args)
    marker_detector = MarkerDetector()
    rclpy.spin(marker_detector)
    marker_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
