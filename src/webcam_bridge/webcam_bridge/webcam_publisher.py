import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.03, self.timer_callback)  # Calling the timer callback so publishing images at 60 Hz (60 fps)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('/dev/video1')  #Default webcam
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open webcam.')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # OpenCV to ROS2 
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(image_msg)
            self.get_logger().info('Published a frame')
        else:
            self.get_logger().error('Failed to capture a frame.')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
