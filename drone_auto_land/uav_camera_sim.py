import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import threading

class MarkerDetector(Node):
    def __init__(self):
        super().__init__('marker_detector')
        self.subscription = self.create_subscription(
            Image,
            'camera',
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # Add your camera matrix and distortion coefficients here
        self.camera_matrix = np.array([[4.970870507466349295e+02, 0.000000000000000000e+00, 3.168387473166100108e+02],
                                       [0.000000000000000000e+00, 4.976011127668800782e+02, 2.342740555042151982e+02],
                                       [0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]])
        self.distortion_coeffs = np.array([2.092787575985481374e-01, -3.700562770349383745e-01, -1.151105913997617098e-03, 1.872085490492018858e-03, 2.131628286804855901e-02])

        # Define the codec and create VideoWriter object
        fourcc = cv.VideoWriter_fourcc(*'XVID')
        self.out = cv.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

        # Flag to indicate if recording is active
        self.is_recording = True

    def image_callback(self, msg):
        # Convert the ROS Image message to a CV Image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Resize
        cv_image = cv.resize(cv_image, (640, 480))

        # Undistort the image
        cv_image = cv.undistort(cv_image, self.camera_matrix, self.distortion_coeffs)

        # Perform marker detection
        aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
        parameters = cv.aruco.DetectorParameters_create()
        corners, ids, _ = cv.aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)

        # If at least one marker detected
        if len(corners) > 0:
            # Draw detected markers on the image
            cv_image = cv.aruco.drawDetectedMarkers(cv_image, corners, ids)

            # Estimate pose of each marker and return the values rvec and tvec
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.distortion_coeffs)

            # Draw axis for the aruco markers
            for i in range(len(rvecs)):
                cv_image = cv.aruco.drawAxis(cv_image, self.camera_matrix, self.distortion_coeffs, rvecs[i], tvecs[i], 0.1)

        # Write the frame to the video file
        if self.is_recording:
            self.out.write(cv_image)

def record_thread(marker_detector):
    input("Press Enter to stop recording...")
    marker_detector.is_recording = False
    marker_detector.out.release()
    marker_detector.destroy_node()
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    marker_detector = MarkerDetector()

    # Start recording thread
    record_thread_instance = threading.Thread(target=record_thread, args=(marker_detector,))
    record_thread_instance.start()

    # Spin the node
    rclpy.spin(marker_detector)

    # Join the recording thread
    record_thread_instance.join()

if __name__ == '__main__':
    main()
