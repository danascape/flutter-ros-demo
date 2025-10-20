#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class DetectionModeController(Node):
    def __init__(self):
        super().__init__('detection_mode_controller')

        # Publishers
        self.mode_publisher = self.create_publisher(String, '/detection_mode', 10)
        self.status_publisher = self.create_publisher(String, '/mode_status', 10)

        # Subscribers - Listen for mode change requests from Flutter
        self.command_subscriber = self.create_subscription(
            String, '/mode_command', self.handle_mode_command, 10)

        # Current state
        self.current_mode = 'driver'  # Default to driver detection (mood monitoring)
        self.available_modes = ['driver', 'front']

        # Publish initial status
        self.publish_mode_status()

        self.get_logger().info("Detection Mode Controller started")
        self.get_logger().info(f"Current mode: {self.current_mode}")
        self.get_logger().info(f"Available modes: {', '.join(self.available_modes)}")

    def handle_mode_command(self, msg):
        """Handle mode change commands from Flutter app"""
        try:
            command_data = json.loads(msg.data)
            requested_mode = command_data.get('mode', self.current_mode)

            if requested_mode in self.available_modes:
                if requested_mode != self.current_mode:
                    self.switch_mode(requested_mode)
                else:
                    self.get_logger().info(f"Already in {requested_mode} mode")
            else:
                self.get_logger().error(f"Invalid mode requested: {requested_mode}")
                self.publish_error_status(f"Invalid mode: {requested_mode}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid command format: {msg.data}")
            self.publish_error_status("Invalid command format")

    def switch_mode(self, new_mode):
        """Switch detection mode and notify all components"""
        old_mode = self.current_mode
        self.current_mode = new_mode

        self.get_logger().info(f"🔄 Switching mode: {old_mode} → {new_mode}")

        # Publish mode change to all detection nodes
        mode_message = {
            'mode': new_mode,
            'previous_mode': old_mode,
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'controller': 'detection_mode_controller'
        }

        msg = String()
        msg.data = json.dumps(mode_message)
        self.mode_publisher.publish(msg)

        # Publish status update
        self.publish_mode_status()

        # Log the change
        if new_mode == 'driver':
            self.get_logger().info("🚗 Driver monitoring mode activated")
            self.get_logger().info("   → Emotion detection")
            self.get_logger().info("   → Fatigue monitoring")
            self.get_logger().info("   → Alertness analysis")
            self.get_logger().info("   → Brake warning if driver sleepy/fatigued")
        elif new_mode == 'front':
            self.get_logger().info("🛣️  Front detection mode activated")
            self.get_logger().info("   → Object detection")
            self.get_logger().info("   → Road safety monitoring")
            self.get_logger().info("   → Collision avoidance")
            self.get_logger().info("   → Brake warning if objects too near")

    def publish_mode_status(self):
        """Publish current mode status"""
        status_data = {
            'current_mode': self.current_mode,
            'available_modes': self.available_modes,
            'status': 'active',
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }

        msg = String()
        msg.data = json.dumps(status_data)
        self.status_publisher.publish(msg)

    def publish_error_status(self, error_message):
        """Publish error status"""
        status_data = {
            'current_mode': self.current_mode,
            'available_modes': self.available_modes,
            'status': 'error',
            'error': error_message,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }

        msg = String()
        msg.data = json.dumps(status_data)
        self.status_publisher.publish(msg)

def main():
    rclpy.init()
    node = DetectionModeController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down detection mode controller...")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()