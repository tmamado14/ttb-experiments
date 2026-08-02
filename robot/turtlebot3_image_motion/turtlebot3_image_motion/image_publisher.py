#!/usr/bin/env python3
"""
Image Publisher Node for TurtleBot3
Captures images from RPi Camera and publishes to /image/compressed topic
Uses JPEG compression for better bandwidth over network
Runs on: TurtleBot3 (Raspberry Pi)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        # Parameters - optimized for quality + speed
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('fps', 30.0)  # 30 FPS for smooth video
        self.declare_parameter('image_topic', '/image/compressed')
        self.declare_parameter('width', 640)  # Higher resolution
        self.declare_parameter('height', 480)
        self.declare_parameter('jpeg_quality', 70)  # Good quality
        self.declare_parameter('rotate_90', True)

        camera_index = self.get_parameter('camera_index').value
        fps = self.get_parameter('fps').value
        image_topic = self.get_parameter('image_topic').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.rotate_90 = self.get_parameter('rotate_90').value

        # Publisher for compressed images
        self.publisher_ = self.create_publisher(CompressedImage, image_topic, 1)

        # Try different camera backends
        self.cap = None

        # Method 1: Try V4L2 backend first (works on most systems)
        self.get_logger().info(f'Trying V4L2 backend for camera {camera_index}...')
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)

        if self.cap.isOpened():
            # Set MJPEG format for faster capture (camera does encoding)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer for lower latency
        else:
            self.get_logger().warn('V4L2 failed, trying default backend...')

            # Method 2: Try default backend
            self.cap = cv2.VideoCapture(camera_index)

            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                self.cap.set(cv2.CAP_PROP_FPS, fps)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera with any backend!')
            self.get_logger().error('Check: ls /dev/video* and libcamera-hello --list-cameras')
        else:
            self.get_logger().info(f'Camera opened successfully!')
            self.get_logger().info(f'Publishing compressed images to {image_topic} at {fps} FPS')
            self.get_logger().info(f'JPEG quality: {self.jpeg_quality}, Rotate 90: {self.rotate_90}')

        # Timer for publishing
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.publish_image)

        self.frame_count = 0
        self.reconnect_attempts = 0

    def publish_image(self):
        if self.cap is None or not self.cap.isOpened():
            self.try_reconnect()
            return

        ret, frame = self.cap.read()
        if ret:
            # Rotate 90 degrees clockwise if enabled (for RPi Camera orientation)
            if self.rotate_90:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

            # Compress to JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            _, compressed = cv2.imencode('.jpg', frame, encode_param)

            # Create CompressedImage message
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            msg.format = "jpeg"
            msg.data = np.array(compressed).tobytes()

            self.publisher_.publish(msg)

            self.frame_count += 1
            self.reconnect_attempts = 0
            if self.frame_count % 100 == 0:
                self.get_logger().info(f'Published {self.frame_count} compressed frames')
        else:
            self.get_logger().warn('Failed to capture image from camera')
            self.try_reconnect()

    def try_reconnect(self):
        """Try to reconnect to camera."""
        self.reconnect_attempts += 1
        if self.reconnect_attempts % 50 == 1:  # Log every 50 attempts
            self.get_logger().warn(f'Attempting camera reconnect (attempt {self.reconnect_attempts})...')

        if self.cap is not None:
            self.cap.release()

        camera_index = self.get_parameter('camera_index').value
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)

        if self.cap.isOpened():
            width = self.get_parameter('width').value
            height = self.get_parameter('height').value
            fps = self.get_parameter('fps').value
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.get_logger().info('Camera reconnected successfully!')
            self.reconnect_attempts = 0

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        self.get_logger().info('Camera released')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
