#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserTimestampPublisher(Node):

    def __init__(self):
        super().__init__('laser_timestamp_publisher')

        # 创建订阅器，订阅雷达数据
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_no_time',
            self.scan_callback,
            10)  # 设置队列大小为10

        # 创建发布器，发布带有时间戳的雷达数据
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',
            10)  # 设置队列大小为10

    def scan_callback(self, msg):
        # 处理接收到的雷达数据，并加上正确的时间戳
        msg.header.stamp = self.get_clock().now().to_msg()

        # 发布带有时间戳的雷达数据
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserTimestampPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
