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

        #debug
        self.aruco_pose_camera_pub = self.create_publisher(PoseStamped, 'aruco_pose_camera', 10)

        self.bridge = CvBridge()
        self.marker_id = 118
        self.embedded_marker_id = 345
        self.marker_size = 0.295
        self.embedded_marker_size = 0.042


        self.marker_bits = np.array([[1, 0, 0, 0, 0],
                                [1, 0, 1, 1, 1],
                                [0, 1, 1, 1, 0],
                                [1, 0, 1, 1, 1],
                                [0, 1, 0, 0, 1]], dtype=np.uint8)
        
        self.embedded_marker_bits = np.array([[1, 0, 1, 1, 1],
                                            [1, 0, 1, 1, 1],
                                            [1, 0, 1, 1, 1],
                                            [0, 1, 0, 0, 1],
                                            [1, 0, 1, 1, 1]], dtype=np.uint8)
                                              

        #debu

        # Load the camera matrix and distortion coefficients
        camera_matrix_file = 'src/drone_auto_land/drone_auto_land/camera_parameters/camera_matrix.txt'
        distortion_coefficients_file = 'src/drone_auto_land/drone_auto_land/camera_parameters/camera_distortion_coefficients.txt'
        self.camera_matrix = np.loadtxt(camera_matrix_file, delimiter=',')
        self.distortion_coeffs = np.loadtxt(distortion_coefficients_file, delimiter=',')
        self.last_aruco_area = None
        self.failed_finding_marker_tries = 0

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
        _, threshold_img_binary = cv.threshold(gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        #adaptive thresholding
       # threshold_value=127
       # _, threshold_img_inv_127 = cv.threshold(gray, threshold_value, 255, cv.THRESH_BINARY_INV)
       # threshold_value = 80
       # _, threshold_img_inv_80 = cv.threshold(gray, threshold_value, 255, cv.THRESH_BINARY_INV)


        #threshold_img = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, threshold_block_size, threshold_constant)
        #debug
        #self.adaptive_threshold_image_debug = threshold_img
        #self.adaptive_threshold_image_pub.publish(self.bridge.cv2_to_imgmsg(threshold_img, encoding='mono8'))
        #self.adaptive_threshold_image_pub_binary.publish(self.bridge.cv2_to_imgmsg(threshold_img_binary, encoding='mono8'))
        #self.adaptive_threshold_image_pub_inv_127.publish(self.bridge.cv2_to_imgmsg(threshold_img_inv_127, encoding='mono8'))
        #self.adaptive_threshold_image_pub_inv_80.publish(self.bridge.cv2_to_imgmsg(threshold_img_inv_80, encoding='mono8'))
        return threshold_img_binary
    
    def angle_vector(self, v1, v2):
        cosine_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        return np.arccos(cosine_angle)
    
    def get_min_max_area(self, threshold_distance_rate=0.5):
        min_area = self.last_aruco_area - self.last_aruco_area * threshold_distance_rate
        max_area = self.last_aruco_area + self.last_aruco_area * threshold_distance_rate
        return min_area, max_area

    def order_corners(self, corners):

        ordered_corners = np.zeros((4, 2), dtype=np.float32)

        s = np.sum(corners, axis=1)
        ordered_corners[0] = corners[np.argmin(s)]
        ordered_corners[2] = corners[np.argmax(s)]

        diff = np.diff(corners, axis=1)
        ordered_corners[1] = corners[np.argmin(diff)]
        ordered_corners[3] = corners[np.argmax(diff)]

        #ordered_corners = ordered_corners.reshape(-1, 1, 2)
        return ordered_corners
    
    def detect_quadrilateral(self, img, min_area=100, max_area=100000, min_angle=0.785, max_angle=2.356):
        blurred_img = cv.GaussianBlur(img, (3, 3), 0)
        #lower_threshold = np.median(blurred_img)
        #upper_threshold = lower_threshold * 1.5
        #edges = cv.Canny(blurred_img, lower_threshold, upper_threshold)
        #publish edges
        #self.edges_pub.publish(self.bridge.cv2_to_imgmsg(edges, encoding='mono8'))

        contours, _ = cv.findContours(blurred_img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        #debug
        #add areas and angles to the contours
       # for contour in contours:
            #add area to self.contours_debug
        #    area = cv.contourArea(contour)
            #if area < 50000:
            #    continue
         #   cv.drawContours(self.contours_debug, [contour], -1, (0, 255, 0), 2)
          #  if (area > 50000):
           #     cv.putText(self.contours_debug, str(area), tuple(contour[0][0]), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        #countours_filtered = []
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
                    ordered_corners = self.order_corners(approx)
                    potential_marker_corners.append(ordered_corners)
                   # countours_filtered.append(contour)

        #debug

        #cv.drawContours(self.countours_filtered_debug, countours_filtered, -1, (0, 255, 0), 2)
        #publish contours_filtered
        #self.contours_pub.publish(self.bridge.cv2_to_imgmsg(self.contours_debug, encoding='bgr8'))
        #self.contours_filtered_pub.publish(self.bridge.cv2_to_imgmsg(self.countours_filtered_debug, encoding='bgr8'))
        return potential_marker_corners
    
    def get_bits(self, img, corners, perspectiveRemoveIgnoredMarginPerCell=0.13):

        dst_pts = np.array([[0, 0], [49, 0], [49, 49], [0, 49]], dtype=np.float32)
        M = cv.getPerspectiveTransform(corners, dst_pts)
        warped = cv.warpPerspective(img, M, (50, 50))
        #self.warped_debug = warped
        #self.warped_pub.publish(self.bridge.cv2_to_imgmsg(warped, encoding='bgr8'))
        
        gray_img = cv.cvtColor(warped, cv.COLOR_BGR2GRAY)

        #Apply otsu thresholding with gaussian filtering
        blured_img = cv.GaussianBlur(gray_img, (3, 3), 0)
        threshold_value = 80
        _, threshold_image = cv.threshold(blured_img, threshold_value, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)
        #_, threshold_image = cv.threshold(blured_img, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        #self.threshold_bits_pub.publish(self.bridge.cv2_to_imgmsg(threshold_image, encoding='mono8'))

        # Perform marker detectiongray
        cell_size = threshold_image.shape[0] // 7
        cell_margin = int(cell_size * perspectiveRemoveIgnoredMarginPerCell)
        bits = np.zeros((7,7), dtype=np.uint8)

        #add a seven by seven grid to the warped and between each cell draw a rectangle removing the cell_margin
        #threshold_image_grid = threshold_image.copy()
        #threshold_image_grid = cv.cvtColor(threshold_image_grid, cv.COLOR_GRAY2BGR)
        #publish

        #for i in range(1, 7):
         #   start_point_vertical = (i * cell_size, 0)
          #  end_point_vertical = (i * cell_size, 50)
           # start_point_horizontal = (0, i * cell_size)
            #end_point_horizontal = (50, i * cell_size)

            #color = (0, 0, 255)
                        # Draw vertical line
            #cv.line(threshold_image_grid, start_point_vertical, end_point_vertical, color, 1)
            
            # Draw horizontal line
            #cv.line(threshold_image_grid, start_point_horizontal, end_point_horizontal, color, 1)

       # self.warped_threshold_pub.publish(self.bridge.cv2_to_imgmsg(threshold_image_grid, encoding='bgr8'))

        for x in range(7):
            for y in range(7):
                cell = threshold_image[x*cell_size:(x+1)*cell_size, y*cell_size:(y+1)*cell_size]
                cell = cell[cell_margin:cell_size-cell_margin, cell_margin:cell_size-cell_margin]
                if np.mean(cell) < 127:
                    bits[x,y] = 1

        #create image with bits and publish to the topic
       # bits_img = np.zeros((50, 50), dtype=np.uint8)
       # bit_cell_size = bits_img.shape[0] // 7
       # for i in range(7):
        #    for j in range(7):
         #       if bits[i, j] == 1:
          #          cv.rectangle(bits_img, 
           #                      (j*bit_cell_size, i*bit_cell_size), 
            #                     ((j+1)*bit_cell_size, (i+1)*bit_cell_size), 
             #                    255, 
              #                   -1)

        #self.bits_image_pub.publish(self.bridge.cv2_to_imgmsg(bits_img, encoding='mono8'))

        return bits
    
    def hamming_distance(self, bits):

        marker_118_hamming_distance = 0
        marker_345_hamming_distance = 0

        for i in range(5):
            for j in range(5):
                marker_118_hamming_distance += bits[i, j] ^ self.marker_bits[i, j]
                marker_345_hamming_distance += bits[i, j] ^ self.embedded_marker_bits[i, j]

        return marker_118_hamming_distance, marker_345_hamming_distance
                
    
    def verify_potential_marker(self, bits, corners, max_erroneous_bits_border=3, max_erroneous_bits_marker=5):
        marker_id = None
        # Check if the border bits are correct
        border_bits = np.concatenate((bits[0, :], bits[-1, :], bits[:, 0], bits[:, -1]))
        erroneous_border_bits = np.sum(border_bits == 1)

        if erroneous_border_bits > max_erroneous_bits_border:
            return marker_id, corners
        

        marker_bits = bits[1:6, 1:6]
        #print(marker_bits)
        #debug
       # marker_img = np.zeros((50, 50), dtype=np.uint8)
       # for i in range(5):
         #   for j in range(5):
         #       if marker_bits[i, j] == 1:
        #            cv.rectangle(marker_img, (j*60, i*60), ((j+1)*60, (i+1)*60), 255, -1)
       # self.marker_bits_debug = marker_img


        # Check if the marker bits are correct
        erroneous_marker_bits, erroneous_embedded_marker_bits = self.hamming_distance(marker_bits)

        for _ in range(4):
            if erroneous_marker_bits <= max_erroneous_bits_marker:
                self.erroneous_bits_marker_debug = erroneous_marker_bits
                marker_id = self.marker_id
                return marker_id, corners
            
            elif erroneous_embedded_marker_bits <= max_erroneous_bits_marker:
                self.erroneous_bits_marker_debug = erroneous_embedded_marker_bits
                marker_id = self.embedded_marker_id
                return marker_id, corners
            else:
                # Rotate the marker 90 degrees
                marker_bits = np.rot90(marker_bits)

                #rotate corners
                corners = np.array([corners[3], corners[0], corners[1], corners[2]], dtype=np.float32)


                erroneous_marker_bits, erroneous_embedded_marker_bits = self.hamming_distance(marker_bits)

        return marker_id, corners
    
    def estimate_pose(self, corners, marker_size):
        ret = cv.aruco.estimatePoseSingleMarkers(corners, marker_size, self.camera_matrix, self.distortion_coeffs)
        tvec = ret[1][0, 0, :]
        return tvec

    def remove_inner_markers(self, marker_corners, minCenterDistance, minCornerDistance):
        marker_centers = [np.mean(corners, axis=0) for corners in marker_corners]
        marker_perimeters = [cv.arcLength(corners, True) for corners in marker_corners]
        keep_indices = []

        for i in range(len(marker_corners)):
            keep = True

            for j in range(len(marker_corners)):
                if i != j: 
                    center_distance = np.linalg.norm(marker_centers[i] - marker_centers[j])
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
    
    def draw_markers(self, img, marker_corners, marker_ids):
        for i in range(len(marker_corners)):
            corners = marker_corners[i][0].astype(np.int32)
            cv.polylines(img, [corners], True, (0, 255, 0), 2)
            bottom_right_corner_id = np.argmax(np.sum(corners, axis=1))

            text_position = tuple(corners[bottom_right_corner_id])
            cv.putText(img, str(marker_ids[i][0]), text_position, cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            #add to the top left corner the area of the marker
           # top_left_corner_id = np.argmin(np.sum(corners, axis=1))
           # area = cv.contourArea(corners)
           # cv.putText(img, str(area), tuple(corners[top_left_corner_id]), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)



        return img
    


    # Callback function for the camera image
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        undistorted_img = cv.undistort(cv_image, self.camera_matrix, self.distortion_coeffs)
        #debug
       # self.contours_debug = undistorted_img.copy()
       # self.countours_filtered_debug = undistorted_img.copy()

        undistorted_img_bin = self.image_binarization(undistorted_img)
        potential_marker_corners = []
        if self.last_aruco_area is not None and self.failed_finding_marker_tries < 10:
            min_area, max_area = self.get_min_max_area()
            #print("min_area: ", min_area, "max_area: ", max_area)
            potential_marker_corners = self.detect_quadrilateral(undistorted_img_bin, min_area, max_area, 0.785, 2.356)
        else:
            potential_marker_corners = self.detect_quadrilateral(undistorted_img_bin, 100, 307200, 0.785, 2.356)
        
        potential_marker_corners = self.remove_inner_markers(potential_marker_corners, 5, 20)
        
        marker_ids = []
        marker_corners = []


        if len(potential_marker_corners) > 0:

            for corners in potential_marker_corners:
                potential_marker_bits = self.get_bits(undistorted_img, corners)
                marker_id, corners_corrected_orientation = self.verify_potential_marker(potential_marker_bits, corners)
                if marker_id is not None:
                    marker_ids.append(marker_id)
                    marker_corners.append(corners_corrected_orientation)
            if (len(marker_corners) > 0):
                marker_ids = np.array(marker_ids, dtype=np.int32).reshape(-1, 1)
                marker_corners = tuple(np.array([corner], dtype=np.float32) for corner in marker_corners)
                undistorted_img = self.draw_markers(undistorted_img, marker_corners, marker_ids)

                tvec = None

                match len(marker_ids):
                    case 1:
                        tvec = self.estimate_pose(marker_corners, (self.marker_size if marker_ids[0][0] == self.marker_id else self.embedded_marker_size))
                        self.last_aruco_area = cv.contourArea(marker_corners[0])
                    case 2:
                        # Check which of the corners is the big marker and estimate its pose
                        if marker_ids[0][0] == self.marker_id:
                            tvec = self.estimate_pose([marker_corners[0]], self.marker_size)
                            self.last_aruco_area = cv.contourArea(marker_corners[0])
                        else:
                            tvec = self.estimate_pose([marker_corners[1]], self.marker_size)
                            self.last_aruco_area = cv.contourArea(marker_corners[1])
                    case _:
                        self.get_logger().info("More than 2 markers detected")

                if tvec is not None:
                    self.publish_aruco_pose(tvec[0], tvec[1], tvec[2])
        if len(marker_ids) == 0:
            self.failed_finding_marker_tries += 1
        else:
            self.failed_finding_marker_tries = 0
        self.aruco_image_pub.publish(self.bridge.cv2_to_imgmsg(undistorted_img, encoding='bgr8'))

        
def main(args=None):
    rclpy.init(args=args)
    marker_detector = MarkerDetector()
    rclpy.spin(marker_detector)
    marker_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()