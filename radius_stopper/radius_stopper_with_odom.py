import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import random


class RadiusStopper(Node):
    def __init__(self):
        super().__init__('radius_stopper_with_odom')

        # Parameter so the radius can be changed without editing code
        self.declare_parameter('stop_radius', 7.0)
        self.stop_radius = self.get_parameter('stop_radius').value

        # Subscriber: listens to the IMU topic
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Publisher: sends velocity commands to the robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer: runs control_loop 10 times per second
        self.timer = self.create_timer(0.1, self.control_loop)

        # State of your estimator
        self.x = 0.0
        self.y = 0.0
        self.stopped = False

    def odom_callback(self, msg: Odometry):
        
        if self.stopped:
            return

        # odom gives exact x and y coordinates relative to initial point
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def control_loop(self):
        cmd = Twist()
        
        r = math.hypot(self.x, self.y)

        if r >= self.stop_radius:
            self.stopped = True

        if self.stopped:
            cmd.linear.x = 0.0 
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = random.uniform(0.3, 0.4)     #for random movement of the robot
            cmd.angular.z = random.uniform(-3.0, 3.0)  

        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'estimated r = {r:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = RadiusStopper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()