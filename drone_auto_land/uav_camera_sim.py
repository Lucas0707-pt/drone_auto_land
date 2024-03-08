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
        self.font = cv.FONT_HERSHEY_PLAIN
        self.marker_size = 20

            
        # Add your camera matrix and distortion coefficients here
        self.camera_matrix = np.array([[4.970870507466349295e+02, 0.000000000000000000e+00, 3.168387473166100108e+02],
                                       [0.000000000000000000e+00, 4.976011127668800782e+02, 2.342740555042151982e+02],
                                       [0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]])
        self.distortion_coeffs = np.array([0, 0, 0, 0, 0])

        # Define the codec and create VideoWriter object
        fourcc = cv.VideoWriter_fourcc(*'XVID')
        self.out = cv.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

    def image_callback(self, msg):
        # Convert the ROS Image message to a CV Image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Resize
        cv_image = cv.resize(cv_image, (640, 480))

        # Undistort the image
        #cv_image = cv.undistort(cv_image, self.camera_matrix, self.distortion_coeffs)
        
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
                #print("Marker ID: ", ids[i], "Center: ", (cx, cy))

            # Estimate pose of each marker and return the values rvec and tvec
            ret = cv.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.distortion_coeffs)
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
            print(str_position)
            #cv.putText(cv_image, str_position, (0, 100), self.font, 1, (0, 255, 0), 2, cv.LINE_AA)
            #cv.aruco.drawAxis(cv_image, self.camera_matrix, self.distortion_coeffs, rvec, tvec, 10)


        #show the image
        cv.imshow("Image window", cv_image)
        cv.waitKey(3)
        

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
