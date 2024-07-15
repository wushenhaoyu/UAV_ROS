#!/usr/bin/env python3
import serial
import struct
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import StaticTransformBroadcaster, TransformStamped


class SerialDataReceiver(Node):
    def __init__(self):
        super().__init__('serial_data_receiver')
        self.get_logger().info('Starting IMU data receiver')
        self.imu_publisher = self.create_publisher(Imu, 'imu', 10)
        self.serial_port = serial.Serial('/dev/imu', 115200, timeout=1)
        self.buffer = bytearray()
        self.receiving = False
        self.timer = self.create_timer(0.001, self.read_and_publish_imu_data)
        
    """    imu_transform = TransformStamped()
        imu_transform.header.stamp = self.get_clock().now().to_msg()
        imu_transform.header.frame_id = 'base_link'
        imu_transform.child_frame_id = 'imu_link'
        imu_transform.transform.translation.x = 0.0
        imu_transform.transform.translation.y = 0.0
        imu_transform.transform.translation.z = 0.0
        imu_transform.transform.rotation.x = 0.0
        imu_transform.transform.rotation.y = 0.0
        imu_transform.transform.rotation.z = 0.0
        imu_transform.transform.rotation.w = 1.0 """
        
        # 发布静态变换



    def read_and_publish_imu_data(self):
        while self.serial_port.in_waiting > 0:
            byte = self.serial_port.read(1)
            if not self.receiving and byte == b'\xAA':
                self.receiving = True
                self.buffer.clear()
                #self.buffer.extend(byte)
            elif self.receiving:
                if byte == b'\xAF':
                    self.receiving = False
                    if len(self.buffer) == 36:
                        self.process_imu_data(self.buffer)
                    self.buffer.clear()
                else:
                    self.buffer.extend(byte)

    def process_imu_data(self, data):
        if len(data) != 36:
            self.get_logger().warn('Received data packet is not the expected length.')
            return
        
        try:
            unpacked_data = struct.unpack('<9f', data)
            imu_msg = Imu()
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            
            # 加速度数据
            imu_msg.linear_acceleration.x = unpacked_data[0] * 0.01
            imu_msg.linear_acceleration.y = unpacked_data[1] * 0.01
            imu_msg.linear_acceleration.z = unpacked_data[2] * 0.01
            
            # 角速度数据
            imu_msg.angular_velocity.x = unpacked_data[3] * 0.01
            imu_msg.angular_velocity.y = unpacked_data[4] * 0.01
            imu_msg.angular_velocity.z = unpacked_data[5] * 0.01
            
            # 欧拉角转换为四元数
            orientation_q = R.from_euler('xyz', [unpacked_data[6]  , unpacked_data[7] , unpacked_data[8] ], degrees=True).as_quat()
            imu_msg.orientation.x = orientation_q[0]
            imu_msg.orientation.y = orientation_q[1]
            imu_msg.orientation.z = orientation_q[2]
            imu_msg.orientation.w = orientation_q[3]

            self.imu_publisher.publish(imu_msg)
        except struct.error:
            self.get_logger().error('Failed to unpack data, unexpected format or length.')


def main(args=None):
    rclpy.init(args=args)
    node = SerialDataReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()