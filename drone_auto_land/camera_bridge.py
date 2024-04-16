import rclpy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node

class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera_bridge')
        self.camera_image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0/30.0, self.camera_image_callback)
        self.cap = cv.VideoCapture(5)

    def release_camera(self):
        self.cap.release()

    def camera_image_callback(self):
        ret, frame = self.cap.read()
        if ret:
            #print current dimensions of the frame
            print(frame.shape)
            #frame = cv.resize(frame, (640, 480))
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.camera_image_pub.publish(msg)
        else:
            self.get_logger().info('No frame')


def main(args=None):
    rclpy.init(args=args)
    camera_bridge = CameraBridge()
    rclpy.spin(camera_bridge)
    camera_bridge.release_camera()
    camera_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()