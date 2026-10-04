#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DownScanner(Node):
    def __init__(self):
        super().__init__('down_scanner')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        self.sub = self.create_subscription(
            Image, '/uav1/camera_down', self.on_image, qos)
        self.bridge = CvBridge()
        self.counter = 0
        self.get_logger().info('Подписался на /uav1/camera_down (best_effort)')

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(f'/tmp/scan_{self.counter:04d}.png', frame)
            self.counter += 1
            self.get_logger().info(f'Кадр {self.counter}: {msg.width}x{msg.height}')
        except Exception as e:
            self.get_logger().error(f'Ошибка: {e}')

def main():
    rclpy.init()
    rclpy.spin(DownScanner())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
