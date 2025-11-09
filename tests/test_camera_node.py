#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import base64

class ImprovedCameraNode(Node):
    def __init__(self):
        super().__init__('test_camera_node')
        self.publisher = self.create_publisher(String, '/camera_images', 10)
        self.timer = self.create_timer(0.2, self.capture_and_publish)  # 5 FPS to avoid overwhelming

        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera")
            return

        # Set camera properties for smaller images to reduce data
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        self.get_logger().info("Camera node started - sending actual image data")

    def capture_and_publish(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Resize frame to reduce data size
        frame = cv2.resize(frame, (320, 240))
        height, width = frame.shape[:2]

        # Convert to JPEG with lower quality to reduce size
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        _, jpeg_data = cv2.imencode('.jpg', frame, encode_param)

        # Convert to base64 string
        image_b64 = base64.b64encode(jpeg_data).decode('utf-8')

        # Format: IMG_DATA:format:width:height:base64_data
        msg = String()
        msg.data = f"IMG_DATA:jpeg:{width}:{height}:{image_b64}"

        self.publisher.publish(msg)

        # Limit debug output
        if hasattr(self, 'frame_count'):
            self.frame_count += 1
        else:
            self.frame_count = 1

        if self.frame_count % 25 == 0:  # Log every 25 frames
            self.get_logger().info(f"Published frame {self.frame_count}: {width}x{height}, {len(image_b64)} bytes encoded")

    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()

def main():
    rclpy.init()
    node = ImprovedCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down camera node...")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
