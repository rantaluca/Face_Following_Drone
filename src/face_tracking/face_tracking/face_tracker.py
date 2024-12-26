#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import cv2

class FaceTrackerNode(Node):
    def __init__(self):
        super().__init__('face_tracker')
        self.bridge = CvBridge()
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        # Subscriber to the image topic
        self.image_sub_ = self.create_subscription(
            Image,
            #'/webcam_image', #webcam
            '/camera/image_raw',

            self.image_callback,
            10
        )

        self.wanted_face_size = 90.0

        self.tracked_faces_publisher_ = self.create_publisher(Image, 'tracked_faces', 10)
        self.target_point_publisher_ = self.create_publisher(PointStamped, 'target_point', 10)
        self.center_point_publisher_ = self.create_publisher(PointStamped, 'camera_centers', 10)
        self.get_logger().info("Face Tracker Node has started")

    def image_callback(self, msg):
        try:
            #  ROS2 Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            #frame = cv2.flip(frame, 1)  # Flip horizontally 
            center_point = PointStamped()
            center_point.header = msg.header
            center_point.point.x = self.wanted_face_size 
            center_point.point.y = frame.shape[1]/2 #frame width
            center_point.point.z = frame.shape[0]/2  #frame height

            self.center_point_publisher_.publish(center_point)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Convertion to grayscale (needed for face detection)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Calling the OpenCV face detector
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        
        if len(faces)>0:
            # search for the face with the largest area

            choosen_face = faces[0]
            for (x, y, w, h) in faces:
                if w*h > choosen_face[2]*choosen_face[3]:
                    choosen_face = (x, y, w, h)
            x, y, w, h = choosen_face

            #center of the image
            cv2.circle(frame, (frame.shape[1]//2, frame.shape[0]//2), 3, (0, 255, 0), 2)
            #center of the face
            cv2.circle(frame, (x+w//2, y+h//2), 3, (0, 0, 255), 2)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            #vector from the center of the image to the center of the face
            cv2.arrowedLine(frame, (frame.shape[1]//2, frame.shape[0]//2), (x+w//2, y+h//2), (0, 255, 0), 2)

            frame_height, frame_width, _ = frame.shape

            point_message = PointStamped()
            point_message.header = msg.header
            point_message.point.x = float(h)
            point_message.point.y = x + w/2
            point_message.point.z = frame_height - y + h/2  
            self.get_logger().info(f"Face detected at ({point_message.point.x}, {point_message.point.y}, {point_message.point.z})")
            self.target_point_publisher_.publish(point_message)
        
        # Converting back to ROS2 Image message
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')    
        # Publishing the image with the detected faces
        self.tracked_faces_publisher_.publish(image_msg)

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()  


def main(args=None):
    rclpy.init(args=args)
    node = FaceTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
