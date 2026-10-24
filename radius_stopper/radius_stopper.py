import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist


class RadiusStopper(Node):
    def __init__(self):
        super().__init__('radius_stopper')

        # Parameter so the radius can be changed without editing code
        self.declare_parameter('stop_radius', 7.0)
        self.stop_radius = self.get_parameter('stop_radius').value

        # Subscriber: listens to the IMU topic
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, qos_profile_sensor_data)

        # Publisher: sends velocity commands to the robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer: runs control_loop 10 times per second
        self.timer = self.create_timer(0.1, self.control_loop)

        # State of your estimator
        self.vx = self.vy = 0.0
        self.x = self.y = 0.0
        self.last_time = None
        self.stopped = False

    def imu_callback(self, msg: Imu):

        if self.stopped:
            self.vx = 0.0
            return

        # TODO 1: compute dt from msg.header.stamp (skip the very first message)
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time
        
        # TODO 2: (optional) get yaw from msg.orientation and rotate
        #         acceleration to the world frame

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y


        # TODO 3: integrate acceleration -> velocity -> position
        self.vx += ax * dt
        self.x += self.vx * dt
        self.vy += ay * dt
        self.y += self.vy * dt

    def control_loop(self):
        cmd = Twist()
        r = math.hypot(self.x, self.y)

        if r >= self.stop_radius:
            self.stopped = True

        if self.stopped:
            cmd.linear.x = 0.0  # stay stopped
            
        else:
            cmd.linear.x = 0.2  # drive forward (m/s)

        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'estimated r = {r:.2f} m')
        


def main(args=None):
    rclpy.init(args=args)
    node = RadiusStopper()
    rclpy.spin(node)  # keeps the node alive and processing callbacks
    node.destroy_node()
    rclpy.shutdown()


#if __name__ == '__main__':
#    main()
