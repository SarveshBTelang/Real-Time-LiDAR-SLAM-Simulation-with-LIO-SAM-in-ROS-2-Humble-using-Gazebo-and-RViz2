#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class SegwayController(Node):

    def __init__(self):
        super().__init__('segway_controller')

        # Received velocity command
        self.current_vel = Twist()

        # Publisher to motor interface or output
        self.cmd_vel_pub = self.create_publisher(Twist, '/motor_cmd_vel', 10)

        # Subscriber to /cmd_vel (from teleop or planner)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

    def cmd_vel_callback(self, msg):
        self.get_logger().debug(f'Received cmd_vel: {msg}')
        self.current_vel = msg

    def timer_callback(self):
        # Publish received velocity to motor command topic
        self.cmd_vel_pub.publish(self.current_vel)

class IMUSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.pitch = 0.0

        self.subscription = self.create_subscription(
            Imu,
            '/imu_plugin/out',
            self.imu_callback,
            10
        )

    def imu_callback(self, data):
        q0 = data.orientation.x
        q1 = data.orientation.y
        q2 = data.orientation.z
        q3 = data.orientation.w

        self.pitch = math.asin(2 * (q0 * q2 - q1 * q3))
        self.get_logger().debug(f"Pitch: {math.degrees(self.pitch):.2f} degrees")

def main(args=None):
    rclpy.init(args=args)

    segway_controller = SegwayController()
    imu_subscriber = IMUSubscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(segway_controller)
    executor.add_node(imu_subscriber)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        segway_controller.destroy_node()
        imu_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
