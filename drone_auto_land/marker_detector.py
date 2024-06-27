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

        self.marker_size = 0.291
        self.h_fov = 0.9439860762623683
        self.w_res = 640

        # Load the camera matrix and distortion coefficients
        camera_matrix_file = 'camera_parameters/camera_matrix.txt'
        distortion_coefficients_file = 'camera_parameters/camera_distortion_coefficients.txt'
        self.camera_matrix = np.loadtxt(camera_matrix_file, delimiter=',')
        self.distortion_coeffs = np.loadtxt(distortion_coefficients_file, delimiter=',')
        self.last_aruco_pose = None

    # Publish the pose of the detected ArUco marker
    def publish_aruco_pose(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        self.aruco_pose_camera_pub.publish(pose)

    def image_binarization(self, img, threshold_block_size=7, threshold_constant=0):
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        threshold_img = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, threshold_block_size, threshold_constant)

        return threshold_img
    
    def angle_vector(self, v1, v2):
        cosine_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        return np.arccos(cosine_angle)
    
    def euclidean_distance(self, tvec):
        return np.linalg.norm(tvec)
    
    def get_min_max_area(self, euc_distance, threshold_distance = 200):
        width_scene = 2 * euc_distance * np.tan(self.h_fov / 2)
        width_marker = self.marker_size * self.w_res/width_scene
        min_area = (width_marker ** 2)  - threshold_distance
        max_area = (width_marker ** 2)  + threshold_distance
        return min_area, max_area

    
    def detect_quadrilateral(self, img, min_area=100, max_area=100000, min_angle=0.785, max_angle=2.356):
        blurred_img = cv.GaussianBlur(img, (3, 3), 0)

        contours, _ = cv.findContours(blurred_img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        potential_marker_corners = []   
        for contour in contours:
            area = cv.contourArea(contour)
            precision = 0.04
            epsilon = precision * cv.arcLength(contour, True)
            approx = cv.approxPolyDP(contour, epsilon, True)
            if (min_area < area < max_area and len(approx) == 4 and cv.isContourConvex(approx)):

                approx = approx.reshape((-1, 2)).astype(np.float32)
                angles = []
                for i in range(4):
                    v1 = approx[(i+1)%4] - approx[i]
                    v2 = approx[(i+2)%4] - approx[(i+1)%4]
                    angles.append(self.angle_vector(v1, v2))
                
                if (all(angle > min_angle and angle < max_angle for angle in angles)):
                    potential_marker_corners.append(approx)

        return potential_marker_corners
    
    def get_bits(self, img, corners, perspectiveRemoveIgnoredMarginPerCell=0.13):
        dst_pts = np.array([[0, 0], [49, 0], [49, 49], [0, 49]], dtype=np.float32)

        corners = corners.astype(np.float32)
        M = cv.getPerspectiveTransform(corners, dst_pts)
        warped = cv.warpPerspective(img, M, (299, 299))
        
        gray_img = cv.cvtColor(warped, cv.COLOR_BGR2GRAY)

        #Apply otsu thresholding with gaussian filtering
        blured_img = cv.GaussianBlur(gray_img, (3, 3), 0)
        _, threshold_image = cv.threshold(blured_img, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)

        # Perform marker detection
        cell_size = threshold_image.shape[0] // 7
        cell_margin = int(cell_size * perspectiveRemoveIgnoredMarginPerCell)
        bits = np.zeros((7,7), dtype=np.uint8)

        for x in range(7):
            for y in range(7):
                cell = threshold_image[x*cell_size:(y+1)*cell_size, y*cell_size:(y+1)*cell_size]
                cell = cell[cell_margin:cell_size-cell_margin, cell_margin:cell_size-cell_margin]
                if np.mean(cell) > 127:
                    bits[x,y] = 1
        return bits
    
    def hamming_distance(self, bits):
        marker_id_100 = [
            [1, 0, 0, 0, 0],
            [1, 0, 1, 1, 1],
            [0, 1, 0, 0, 1],
            [1, 0, 1, 1, 1],
            [1, 0, 0, 0, 0]
        ]

        sum = 0

        for i in range(5):
            for j in range(5):
                sum += bits[i, j] ^ marker_id_100[i][j]

        return sum
                
    
    def verify_potential_marker(self, bits, maxErroneousBitsInBorderRate, maxErroneousBitsInMarkerRate):

        total_bits = bits.shape[0] * bits.shape[1]

        max_erroneous_bits_border = total_bits * maxErroneousBitsInBorderRate
        max_erroneous_bits_marker = total_bits * maxErroneousBitsInMarkerRate

        # Check if the border bits are correct
        border_bits = np.concatenate((bits[0, :], bits[-1, :], bits[:, 0], bits[:, -1]))
        erroneous_border_bits = np.sum(border_bits == 1)

        if erroneous_border_bits > max_erroneous_bits_border:
            return False, None
        
        marker_bits = bits[1:6, 1:6]

        # Check if the marker bits are correct
        erroneous_marker_bits = self.hamming_distance(marker_bits)

        if erroneous_marker_bits <= max_erroneous_bits_marker:
            return True
    
    def estimate_pose(self, corners, marker_size):
        ret = cv.aruco.estimatePoseSingleMarkers(corners, marker_size, self.camera_matrix, self.distortion_coeffs)
        _, tvec = ret[0][0,0,:], ret[1][0,0,:]
        return tvec

    def remove_inner_markers(self, marker_corners, marker_ids, minCenterDistance, minCornerDistance):
        marker_centers = [np.mean(corners, axis=0) for corners in marker_corners]
        marker_perimeters = [cv.arcLength(corners, True) for corners in marker_corners]
        keep_indices = []

        for i in range(len(marker_corners)):
            keep = True

            for j in range(len(marker_corners)):
                if i != j: 
                    # Calculate the distance between the centers of the markers
                    center_distance = np.linalg.norm(marker_centers[i] - marker_centers[j])
                    # Calculate the minimum distance between the corners of the markers
                    corner_distances = [np.linalg.norm(corner_i - corner_j) for corner_i in marker_corners[i] for corner_j in marker_corners[j]]
                    min_corner_distance = min(corner_distances)

                    if center_distance < minCenterDistance and min_corner_distance < minCornerDistance:
                        if marker_perimeters[i] < marker_perimeters[j]:
                            keep = False
                            break

            if keep:
                keep_indices.append(i)

        marker_corners = [marker_corners[i] for i in keep_indices]

        return marker_corners
    
    def draw_markers(self, img, marker_corners):
        for i in range(len(marker_corners)):
            corners = marker_corners[i][0].astype(np.int32)
            cv.polylines(img, [corners], True, (0, 255, 0), 2)

        return img


    # Callback function for the camera image
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        undistorted_img = cv.undistort(cv_image, self.camera_matrix, self.distortion_coeffs)
        undistorted_img_bin = self.image_binarization(undistorted_img)
        potential_marker_corners = []
        if self.last_aruco_pose is not None:
            euc_distance = self.euclidean_distance(self.last_aruco_pose)
            min_area, max_area = self.get_min_max_area(euc_distance)
            potential_marker_corners = self.detect_quadrilateral(undistorted_img_bin, min_area, max_area, 1.40, 1.75)
        else:
            potential_marker_corners = self.detect_quadrilateral(undistorted_img_bin, 100, 1000000, 1.40, 1.75)

        marker_ids = []
        marker_corners = []

        if len(potential_marker_corners) > 0:

            for corners in potential_marker_corners:
                potential_marker_bits = self.get_bits(undistorted_img, corners)
                is_marker = self.verify_potential_marker(potential_marker_bits, 0.1, 0.1)
                if is_marker:
                    marker_corners.append(corners)
            marker_corners = self.remove_inner_markers(marker_corners, 5, 20)
            if (len(marker_corners) > 0):
                marker_corners = tuple(np.array([corner], dtype=np.float32) for corner in marker_corners)

                undistorted_img = self.draw_markers(undistorted_img, marker_corners)

                tvec = None
                match len(marker_corners):
                    case 1:
                        tvec = self.estimate_pose(marker_corners, self.marker_size)
                    case _:
                        self.get_logger().info("More than 1 marker detected")

                if tvec is not None:
                    self.publish_aruco_pose(tvec[0], tvec[1], tvec[2])
                    self.last_aruco_pose = tvec

                    aruco_pose_camera_text = f"ArUco Pose Camera: x={tvec[0]:.2f}, y={tvec[1]:.2f}, z={tvec[2]:.2f}"
                    cv.putText(undistorted_img, aruco_pose_camera_text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        self.aruco_image_pub.publish(self.bridge.cv2_to_imgmsg(undistorted_img, encoding='bgr8'))

        
def main(args=None):
    rclpy.init(args=args)
    marker_detector = MarkerDetector()
    rclpy.spin(marker_detector)
    marker_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()