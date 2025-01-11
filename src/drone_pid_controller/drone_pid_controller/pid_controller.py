import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist

class PidCmdVelNode(Node):
    def __init__(self):
        super().__init__('pid_cmdvel')

        # Subscriber to face point topic
        self.point_subscriber = self.create_subscription(
            PointStamped,
            '/target_point',
            self.point_callback_pid,
            10
        )

        self.center_point_subscriber = self.create_subscription(
            PointStamped,
            '/camera_centers',
            self.center_point_callback,
            10
        )

        # Publisher on cmd vel
        self.twist_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Publisher on cmd vel
        self.error_publisher = self.create_publisher(
            PointStamped,
            '/error_publisher',
            10
        )

        # Control parameters
        # self.kp = 0.3
        # self.ki = 0.05
        # self.kd = 0.001

        self.kp_x = 0.04
        self.ki_x = 0.003
        self.kd_x = 0.00001

        self.kp_y = 0.4
        self.ki_y = 0.005
        self.kd_y = 0.0001

        self.kp_z = 0.01
        self.ki_z = 0.0001
        self.kd_z = 0.0000001

        self.previous_time = self.get_clock().now()

        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0

        self.previous_error_x = 0.0
        self.previous_error_y = 0.0
        self.previous_error_z = 0.0

        self.center_point = PointStamped()

        self.get_logger().info('PointToCmdVelNode started.')

    def center_point_callback(self, msg):  
        self.center_point = msg
    
    def point_callback_pid(self, msg):

        actual_time = self.get_clock().now()
        dt = (actual_time - self.previous_time).nanoseconds / 1e9
        self.previous_time = actual_time

        # Normalizing error
        error_x = (self.center_point.point.x - msg.point.x)/self.center_point.point.x if self.center_point.point.x != 0 else 0.0
        error_y = (self.center_point.point.y - msg.point.y)/self.center_point.point.y if self.center_point.point.y != 0 else 0.0
        error_z = (self.center_point.point.z - msg.point.z)/self.center_point.point.z if self.center_point.point.y != 0 else 0.0
        print(f"Error: {error_x}, {error_y}, {error_z}")

        error_message = PointStamped()
        error_message.header = msg.header
        error_message.point.x = error_x
        error_message.point.y = error_y
        error_message.point.z = error_z
        self.error_publisher.publish(error_message)

        # Calculation of the PID terms

        proportional_x = error_x * self.kp_x
        proportional_y = error_y * self.kp_y
        proportional_z = error_z * self.kp_z

        self.integral_x = (self.integral_x + (error_x * dt)) *self.ki_x
        self.integral_y = (self.integral_y + (error_y * dt) )*self.ki_y
        self.integral_z = (self.integral_z + (error_z * dt) )*self.ki_z

        derivative_x = ((error_x - self.previous_error_x ) / dt )*self.kd_x if dt > 0 else 0.0
        derivative_y = ((error_y - self.previous_error_y ) / dt)*self.kd_y if dt > 0 else 0.0
        derivative_z = ((error_z - self.previous_error_z ) / dt)*self.kd_z if dt > 0 else 0.0

        # Cmd Output
        twist_msg = Twist()
        if (abs(error_x) > 0.1):
            forward_vel = proportional_x + self.integral_x + derivative_x 
            forward_vel = max(min(forward_vel*5.0, 5.0), -5.0)  # Clamping to [-100, 100]
            twist_msg.linear.x = forward_vel
        if (abs(error_y) > 0.1):
            angular_vel = proportional_y + self.integral_y + derivative_y
            angular_vel = max(min(angular_vel*5.0, 5.0), -5.0)  # Clamping to angular range [-1, 1] 
            twist_msg.angular.z = angular_vel
        if (abs(error_z) > 0.1):
            height_vel = proportional_z + self.integral_z + derivative_z
            height_vel = max(min(height_vel*5.0, 5.0), -5.0)  # Clamp same way as x
            twist_msg.linear.z = height_vel

        self.twist_publisher.publish(twist_msg)



        

def main(args=None):
    rclpy.init(args=args)
    node = PidCmdVelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
