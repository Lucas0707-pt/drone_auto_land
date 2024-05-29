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
        self.camera_image_sub = self.create_subscription(Image, '/v4l/camera/image_raw', self.image_callback, 10)
        self.aruco_image_pub = self.create_publisher(Image, 'aruco_image_aux', 10)
        self.aruco_pose_camera_pub = self.create_publisher(PoseStamped, 'aruco_pose_camera', 10)

        self.bridge = CvBridge()

        self.marker_id = 29
        self.embedded_marker_id = 33
        self.marker_bits = np.array([[1, 1, 0, 0, 1, 0, 1],
                                        [1, 1, 0, 1, 1, 0, 1],
                                        [0, 1, 1, 1, 0, 1, 1],
                                        [1, 0, 1, 1, 1, 0, 0],
                                        [1, 0, 1, 1, 1, 1, 0],
                                        [1, 1, 0, 1, 1, 0, 0],
                                        [0, 1, 1, 0, 0, 0, 0]],dtype=np.uint8)
        
        self.embedded_marker_bits = np.array([[0, 0, 1, 1, 1, 0, 1],
                                              [0, 0, 1, 0, 0, 0, 1],
                                              [1, 0, 0, 0, 1, 0, 0],
                                              [1, 0, 0, 1, 1, 1, 0],
                                              [1, 1, 1, 0, 1, 0, 0],
                                              [0, 0, 1, 0, 0, 1, 1],
                                              [1, 0, 1, 1, 1, 0, 1]], dtype=np.uint8)

        self.marker_size = 0.291
        self.embedded_marker_size = 0.033

        # Load the camera matrix and distortion coefficients
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

    def image_binarization(self, img):
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        _, threshold_img = cv.threshold(gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)

        return threshold_img
    
    def angle_vector(self, v1, v2):
        cosine_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        return np.arccos(cosine_angle)
    
    def detect_corners(self, img, img_aux, min_area, max_area, min_angle, max_angle):
        blurred = cv.GaussianBlur(img, (3, 3), 0)
        lower_threshold = np.median(blurred)
        upper_threshold = lower_threshold * 1.5
        edges = cv.Canny(blurred, lower_threshold, upper_threshold)

        contours, _ = cv.findContours(edges.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        potential_marker_corners = []
        for contour in contours:
            area = cv.contourArea(contour)
            precision = 0.04
            epsilon = precision * cv.arcLength(contour, True)
            approx = cv.approxPolyDP(contour, epsilon, True)
            if (min_area < area < max_area and len(approx) == 4):
                approx = approx.reshape((-1, 2)).astype(np.float32)
                angles = []
                for i in range(4):
                    v1 = approx[(i+1)%4] - approx[i]
                    v2 = approx[(i+2)%4] - approx[(i+1)%4]
                    angles.append(self.angle_vector(v1, v2))
                
                if (all(angle > min_angle and angle < max_angle for angle in angles) or True):
                    potential_marker_corners.append(approx)
                    cv.drawContours(img_aux, [contour], -1, (0, 255, 0), 3)
                    cv.putText(img_aux, f"{area:.2f}", tuple(map(int, approx[0])), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        return potential_marker_corners
    
    def get_bits(self, img, corners, perspectiveRemoveIgnoredMarginPerCell=0.13):
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
        return bits
    
    def verify_potential_marker(self, bits, maxErroneousBitsInBorderRate, maxErroneousBitsInMarkerRate):

        marker_id = None

        total_bits = bits.shape[0] * bits.shape[1]

        max_erroneous_bits_border = total_bits * maxErroneousBitsInBorderRate
        max_erroneous_bits_marker = total_bits * maxErroneousBitsInMarkerRate

        # Check if the border bits are correct
        border_bits = np.concatenate((bits[0, :], bits[-1, :], bits[:, 0], bits[:, -1]))
        erroneous_border_bits = np.sum(border_bits == 1)

        if erroneous_border_bits > max_erroneous_bits_border:
            return False, None
        
        #Check if the marker bits are correct
        marker_bits = bits[1:8, 1:8]
        erroneous_marker_bits = np.sum(marker_bits != self.marker_bits)
        erroneous_embedded_marker_bits = np.sum(marker_bits != self.embedded_marker_bits)

        #Check if the marker bits are correct in all rotations
        for _ in range(4):
            if erroneous_marker_bits <= max_erroneous_bits_marker:
                marker_id = self.marker_id
                break
            if erroneous_embedded_marker_bits <= max_erroneous_bits_marker:
                marker_id = self.embedded_marker_id
                break

            marker_bits = np.rot90(marker_bits)
            erroneous_marker_bits = np.sum(marker_bits != self.marker_bits)
            erroneous_embedded_marker_bits = np.sum(marker_bits != self.embedded_marker_bits)


        return marker_id is not None, marker_id
        

        

    
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

        marker_ids = [marker_ids[i] for i in keep_indices]

        return marker_corners, marker_ids
    
    def draw_markers(self, img, marker_corners, marker_ids):
        for i in range(len(marker_corners)):
            corners = marker_corners[i][0].astype(np.int32)
            cv.polylines(img, [corners], True, (0, 255, 0), 2)

            bottom_right_corner_id = np.argmax(np.sum(corners, axis=1))

            text_position = tuple(corners[bottom_right_corner_id])
            cv.putText(img, str(marker_ids[i][0]), text_position, cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        return img


    # Callback function for the camera image
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        undistorted_img = cv.undistort(cv_image, self.camera_matrix, self.distortion_coeffs)
        undistorted_img_bin = self.image_binarization(undistorted_img)
        undistorted_img_copy = undistorted_img.copy()
        potential_marker_corners = self.detect_corners(undistorted_img_bin, undistorted_img_copy, 1000, 1000000, 1.40,1.75)

        marker_ids = []
        marker_corners = []

        if len(potential_marker_corners) > 0:

            for corners in potential_marker_corners:
                potential_marker_bits = self.get_bits(undistorted_img, corners)
                is_marker, marker_id = self.verify_potential_marker(potential_marker_bits, 0.05, 0.05)
                if is_marker:
                    marker_ids.append(marker_id)
                    marker_corners.append(corners)
            marker_corners, marker_ids = self.remove_inner_markers(marker_corners, marker_ids, 5, 20)
            if (len(marker_ids) > 0):
                marker_corners = tuple(np.array([corner], dtype=np.float32) for corner in marker_corners)
                marker_ids = np.array(marker_ids, dtype=np.int32).reshape(-1, 1)

                undistorted_img = self.draw_markers(undistorted_img, marker_corners, marker_ids)

                tvec = None
                match len(marker_ids):
                    case 1:
                        tvec = self.estimate_pose(marker_corners, (self.marker_size if marker_ids[0][0] == self.marker_id else self.embedded_marker_size))
                    case 2:
                        if marker_ids[0][0] == self.marker_id:
                            tvec = self.estimate_pose([marker_corners[0]], self.marker_size)
                        else:
                            tvec = self.estimate_pose([marker_corners[1]], self.embedded_marker_size)
                    case _:
                        self.get_logger().info("More than 2 markers detected")

                if tvec is not None:
                    self.publish_aruco_pose(tvec[0], tvec[1], tvec[2])

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