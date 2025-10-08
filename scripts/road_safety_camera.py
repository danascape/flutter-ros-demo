#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import base64
import numpy as np
import time
from threading import Lock
import json

# Try to import YOLO - install with: pip install ultralytics
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
    print("YOLO available for real object detection")
except ImportError:
    YOLO_AVAILABLE = False
    print("YOLO not available, using mock detection")

class RoadSafetyCameraNode(Node):
    def __init__(self):
        super().__init__('road_safety_camera')

        # Publishers
        self.image_publisher = self.create_publisher(String, '/camera_images', 10)
        self.detection_publisher = self.create_publisher(String, '/object_detections', 10)

        # Timers for different rates
        self.image_timer = self.create_timer(0.1, self.capture_and_publish_image)  # 10 FPS
        self.detection_timer = self.create_timer(0.2, self.detect_and_publish)     # 5 FPS detection

        # Camera setup
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera")
            return

        # Higher resolution for better detection
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # Thread safety
        self.frame_lock = Lock()
        self.current_frame = None

        # Initialize YOLO model if available
        self.yolo_available = YOLO_AVAILABLE
        if self.yolo_available:
            try:
                self.model = YOLO('yolov8n.pt')  # Nano version for speed
                # Force CPU mode to avoid CUDA errors
                self.model.to('cpu')
                self.get_logger().info("YOLO model loaded successfully (CPU mode)")
            except Exception as e:
                self.get_logger().warn(f"Failed to load YOLO: {e}, using mock detection")
                self.yolo_available = False

        # Road-relevant object classes (COCO dataset indices)
        self.road_objects = {
            0: 'person',
            1: 'bicycle',
            2: 'car',
            3: 'motorcycle',
            5: 'bus',
            7: 'truck',
            9: 'traffic light',
            11: 'stop sign',
            # Add more relevant classes as needed
        }

        # Distance estimation parameters
        self.focal_length = 800  # Approximate focal length in pixels
        self.real_object_heights = {
            'person': 1.7,      # meters
            'car': 1.5,         # meters
            'truck': 3.0,       # meters
            'bus': 3.2,         # meters
            'bicycle': 1.0,     # meters
            'motorcycle': 1.2,  # meters
        }

        self.frame_count = 0
        self.get_logger().info("Road Safety Camera Node started")

    def capture_and_publish_image(self):
        """Capture and publish camera images at high frame rate"""
        ret, frame = self.cap.read()
        if not ret:
            return

        with self.frame_lock:
            self.current_frame = frame.copy()

        # Compress for transmission
        frame_small = cv2.resize(frame, (480, 360))  # Smaller for smooth streaming
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60]
        _, jpeg_data = cv2.imencode('.jpg', frame_small, encode_param)

        # Convert to base64
        image_b64 = base64.b64encode(jpeg_data).decode('utf-8')

        # Publish image
        msg = String()
        msg.data = f"IMG_DATA:jpeg:480:360:{image_b64}"
        self.image_publisher.publish(msg)

    def detect_and_publish(self):
        """Run object detection and publish results"""
        with self.frame_lock:
            if self.current_frame is None:
                return
            frame = self.current_frame.copy()

        if self.yolo_available:
            detections = self.detect_objects_yolo(frame)
        else:
            detections = self.detect_objects_mock(frame)

        # Process each detection
        for detection in detections:
            self.publish_detection(detection)

    def detect_objects_yolo(self, frame):
        """Real YOLO object detection"""
        try:
            # Run inference on CPU with explicit device setting
            results = self.model(frame, verbose=False, device='cpu')
            detections = []

            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Get box coordinates and confidence
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        conf = float(box.conf[0])
                        cls = int(box.cls[0])

                        # Only process road-relevant objects with high confidence
                        if cls in self.road_objects and conf > 0.4:
                            class_name = self.road_objects[cls]

                            # Calculate distance estimate
                            box_height = y2 - y1
                            distance = self.estimate_distance(class_name, box_height)

                            # Determine danger level
                            danger_level = self.assess_danger(class_name, distance, conf)

                            detections.append({
                                'class': class_name,
                                'confidence': conf,
                                'bbox': (int(x1), int(y1), int(x2-x1), int(y2-y1)),
                                'distance': distance,
                                'danger_level': danger_level
                            })

            return detections
        except Exception as e:
            self.get_logger().error(f"YOLO detection error: {e}")
            return []

    def detect_objects_mock(self, frame):
        """Mock detection for testing when YOLO unavailable"""
        import random

        # Simulate realistic road detections
        detections = []
        if random.random() < 0.3:  # 30% chance of detection per frame
            classes = ['car', 'person', 'truck', 'bicycle']
            class_name = random.choice(classes)

            # Random but realistic positioning
            x = random.randint(50, 400)
            y = random.randint(100, 300)
            w = random.randint(80, 200)
            h = random.randint(60, 150)

            conf = random.uniform(0.5, 0.95)
            distance = random.uniform(5, 50)  # 5-50 meters
            danger_level = self.assess_danger(class_name, distance, conf)

            detections.append({
                'class': class_name,
                'confidence': conf,
                'bbox': (x, y, w, h),
                'distance': distance,
                'danger_level': danger_level
            })

        return detections

    def estimate_distance(self, class_name, pixel_height):
        """Estimate distance based on object size in pixels"""
        if class_name not in self.real_object_heights:
            return None

        real_height = self.real_object_heights[class_name]

        # Distance = (Real Height * Focal Length) / Pixel Height
        if pixel_height > 0:
            distance = (real_height * self.focal_length) / pixel_height
            return round(distance, 1)
        return None

    def assess_danger(self, class_name, distance, confidence):
        """Assess danger level based on object type, distance, and confidence"""
        if distance is None:
            return 'unknown'

        # Different danger thresholds for different objects
        if class_name in ['car', 'truck', 'bus']:
            if distance < 10:
                return 'critical'
            elif distance < 20:
                return 'warning'
            elif distance < 40:
                return 'caution'
        elif class_name in ['person', 'bicycle', 'motorcycle']:
            if distance < 8:
                return 'critical'
            elif distance < 15:
                return 'warning'
            elif distance < 30:
                return 'caution'

        return 'safe'

    def publish_detection(self, detection):
        """Publish detection result"""
        # Enhanced format: DET:class|confidence|x,y,w,h|distance|danger_level
        bbox = detection['bbox']
        distance_str = f"{detection['distance']:.1f}m" if detection['distance'] else "unknown"

        detection_data = (f"{detection['class']}|{detection['confidence']:.2f}|"
                         f"{bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}|"
                         f"{distance_str}|{detection['danger_level']}")

        msg = String()
        msg.data = f"DET:{detection_data}"
        self.detection_publisher.publish(msg)

        # Log critical detections
        if detection['danger_level'] == 'critical':
            self.get_logger().warn(f"CRITICAL: {detection['class']} at {distance_str}!")

    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()

def main():
    rclpy.init()
    node = RoadSafetyCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down road safety camera...")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()