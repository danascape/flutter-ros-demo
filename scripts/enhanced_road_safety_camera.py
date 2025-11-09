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

class EnhancedRoadSafetyCameraNode(Node):
    def __init__(self):
        super().__init__('enhanced_road_safety_camera')

        # Publishers
        self.image_publisher = self.create_publisher(String, '/camera_images', 10)
        self.detection_publisher = self.create_publisher(String, '/object_detections', 10)

        # Subscribers for mode switching
        self.mode_subscriber = self.create_subscription(
            String, '/detection_mode', self.handle_mode_change, 10)

        # Detection mode state
        self.current_mode = 'driver'  # Default to driver detection (mood monitoring)
        self.is_active = True  # Always active, but behavior changes based on mode

        # Timers for different rates (reduced to prevent stream blocking)
        self.image_timer = self.create_timer(0.15, self.capture_and_publish_image)  # ~6.5 FPS (reduced from 10)
        self.detection_timer = self.create_timer(0.3, self.detect_and_publish)      # ~3.3 FPS (reduced from 5)

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
            12: 'parking meter',
            # Add more relevant classes as needed
        }

        # Driver-specific object classes for when in driver mode
        self.driver_objects = {
            67: 'cell phone',  # Phone detection for distracted driving
            # Add more driver-relevant objects
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
            'traffic light': 4.0, # meters
            'stop sign': 2.5,   # meters
        }

        self.frame_count = 0
        self.get_logger().info("Enhanced Road Safety Camera Node started")
        self.get_logger().info(f"Current detection mode: {self.current_mode}")

    def handle_mode_change(self, msg):
        """Handle detection mode changes"""
        try:
            mode_data = json.loads(msg.data)
            new_mode = mode_data.get('mode', 'front')

            if new_mode != self.current_mode:
                old_mode = self.current_mode
                self.current_mode = new_mode

                self.get_logger().info(f"  Camera mode switched: {old_mode} → {new_mode}")

                # Front mode does road object detection
                if new_mode == 'front':
                    self.get_logger().info("  Front detection: monitoring road objects and pedestrians")
                    self.publish_system_message("Front Detection: Road object detection active")
                elif new_mode == 'driver':
                    self.get_logger().info("  Driver mode: emotion/mood detection (handled by driver_mood_detection.py)")
                    self.publish_system_message("Driver Detection: Mood detection active")

        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid mode message: {msg.data}")

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
        """Run object detection and publish results based on current mode"""
        with self.frame_lock:
            if self.current_frame is None:
                return
            frame = self.current_frame.copy()

        # Front mode does road object/person detection
        if self.current_mode == 'front':
            # Person/object detection for road safety
            if self.yolo_available:
                detections = self.detect_road_objects_yolo(frame)
            else:
                detections = self.detect_road_objects_mock(frame)

            # Process each detection
            for detection in detections:
                self.publish_road_detection(detection)

        elif self.current_mode == 'driver':
            # Driver mode - mood detection is handled by driver_mood_detection.py
            # This camera just provides the stream
            pass

    def detect_road_objects_yolo(self, frame):
        """Real YOLO object detection for road objects"""
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
                            danger_level = self.assess_road_danger(class_name, distance, conf)

                            detections.append({
                                'class': class_name,
                                'confidence': conf,
                                'bbox': (int(x1), int(y1), int(x2-x1), int(y2-y1)),
                                'distance': distance,
                                'danger_level': danger_level
                            })

            return detections
        except Exception as e:
            self.get_logger().error(f"YOLO road detection error: {e}")
            return []

    def detect_driver_objects_yolo(self, frame):
        """YOLO detection focused on driver-relevant objects"""
        try:
            results = self.model(frame, verbose=False, device='cpu')
            detections = []

            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        conf = float(box.conf[0])
                        cls = int(box.cls[0])

                        # Look for driver-relevant objects (phone, etc.)
                        if cls in self.driver_objects and conf > 0.3:
                            class_name = self.driver_objects[cls]

                            # Phone detection is critical for driver safety
                            danger_level = 'critical' if class_name == 'cell phone' else 'warning'

                            detections.append({
                                'class': class_name,
                                'confidence': conf,
                                'bbox': (int(x1), int(y1), int(x2-x1), int(y2-y1)),
                                'danger_level': danger_level,
                                'context': 'driver_distraction'
                            })

                        # Also detect basic objects that might affect driver view
                        elif cls in self.road_objects and conf > 0.6:
                            class_name = self.road_objects[cls]
                            box_height = y2 - y1
                            distance = self.estimate_distance(class_name, box_height)

                            if distance and distance < 20:  # Close objects are relevant
                                detections.append({
                                    'class': class_name,
                                    'confidence': conf,
                                    'bbox': (int(x1), int(y1), int(x2-x1), int(y2-y1)),
                                    'distance': distance,
                                    'danger_level': 'caution',
                                    'context': 'driver_awareness'
                                })

            return detections
        except Exception as e:
            self.get_logger().error(f"YOLO driver detection error: {e}")
            return []

    def detect_road_objects_mock(self, frame):
        """Mock detection for road objects when YOLO unavailable"""
        import random

        detections = []
        if random.random() < 0.3:  # 30% chance of detection per frame
            classes = ['car', 'person', 'truck', 'bicycle', 'traffic light']
            class_name = random.choice(classes)

            # Random but realistic positioning
            x = random.randint(50, 400)
            y = random.randint(100, 300)
            w = random.randint(80, 200)
            h = random.randint(60, 150)

            conf = random.uniform(0.5, 0.95)
            distance = random.uniform(5, 50)  # 5-50 meters
            danger_level = self.assess_road_danger(class_name, distance, conf)

            detections.append({
                'class': class_name,
                'confidence': conf,
                'bbox': (x, y, w, h),
                'distance': distance,
                'danger_level': danger_level
            })

        return detections

    def detect_driver_objects_mock(self, frame):
        """Mock detection for driver objects"""
        import random

        detections = []

        # Occasionally detect phone usage (distracted driving)
        if random.random() < 0.05:  # 5% chance
            detections.append({
                'class': 'cell phone',
                'confidence': random.uniform(0.6, 0.9),
                'bbox': (random.randint(200, 300), random.randint(150, 250), 50, 100),
                'danger_level': 'critical',
                'context': 'driver_distraction'
            })

        # Sometimes detect objects in driver's field of view
        if random.random() < 0.1:  # 10% chance
            classes = ['car', 'person']
            class_name = random.choice(classes)
            distance = random.uniform(5, 20)

            detections.append({
                'class': class_name,
                'confidence': random.uniform(0.6, 0.9),
                'bbox': (random.randint(100, 400), random.randint(100, 300), 100, 80),
                'distance': distance,
                'danger_level': 'caution',
                'context': 'driver_awareness'
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

    def assess_road_danger(self, class_name, distance, confidence):
        """Assess danger level for road objects with brake warning thresholds"""
        if distance is None:
            return 'unknown'

        # Different danger thresholds for different objects
        # CRITICAL = BRAKE WARNING (object is very near)
        if class_name in ['car', 'truck', 'bus']:
            if distance < 10:  # Less than 10 meters - BRAKE WARNING
                self.get_logger().warn(f"  BRAKE WARNING: {class_name} at {distance:.1f}m!")
                return 'critical'
            elif distance < 20:  # 10-20 meters - warning zone
                return 'warning'
            elif distance < 40:  # 20-40 meters - caution zone
                return 'caution'
        elif class_name in ['person', 'bicycle', 'motorcycle']:
            if distance < 8:  # Less than 8 meters - BRAKE WARNING for vulnerable road users
                self.get_logger().warn(f"  BRAKE WARNING: {class_name} at {distance:.1f}m!")
                return 'critical'
            elif distance < 15:  # 8-15 meters - warning zone
                return 'warning'
            elif distance < 30:  # 15-30 meters - caution zone
                return 'caution'
        elif class_name in ['traffic light', 'stop sign']:
            if distance < 5:
                return 'warning'
            else:
                return 'info'

        return 'safe'

    def publish_road_detection(self, detection):
        """Publish road object detection result with brake warnings"""
        # Enhanced format: DET:class|confidence|x,y,w,h|distance|danger_level
        bbox = detection['bbox']
        distance_str = f"{detection['distance']:.1f}m" if detection['distance'] else "unknown"

        detection_data = (f"{detection['class']}|{detection['confidence']:.2f}|"
                         f"{bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}|"
                         f"{distance_str}|{detection['danger_level']}")

        msg = String()
        msg.data = f"DET:{detection_data}"
        self.detection_publisher.publish(msg)

        # Only log critical detections to reduce console spam
        if detection['danger_level'] == 'critical':
            self.get_logger().warn(f"  BRAKE: {detection['class']} at {distance_str}")
            # Publish additional safety alert for critical near objects
            self.publish_brake_alert(detection['class'], distance_str)

    def publish_driver_object_detection(self, detection):
        """Publish driver-specific object detection"""
        bbox = detection['bbox']
        context = detection.get('context', 'driver')

        if 'distance' in detection:
            distance_str = f"{detection['distance']:.1f}m"
            detection_data = (f"{detection['class']}|{detection['confidence']:.2f}|"
                             f"{bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}|"
                             f"{distance_str}|{detection['danger_level']}|{context}")
        else:
            detection_data = (f"{detection['class']}|{detection['confidence']:.2f}|"
                             f"{bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}|"
                             f"unknown|{detection['danger_level']}|{context}")

        msg = String()
        msg.data = f"DET_DRIVER_OBJ:{detection_data}"
        self.detection_publisher.publish(msg)

        # Log important driver-related detections
        if detection['danger_level'] in ['critical', 'warning']:
            self.get_logger().warn(f"DRIVER: {detection['class']} detected - {detection['danger_level'].upper()}")

    def publish_system_message(self, message):
        """Publish system message for UI"""
        msg = String()
        msg.data = f"SYSTEM:{message}"
        self.detection_publisher.publish(msg)

    def publish_brake_alert(self, object_class, distance):
        """Publish brake alert for critical near objects"""
        msg = String()
        msg.data = f"BRAKE_ALERT:front|{object_class}|{distance}|Object too close - brake immediately!"
        self.detection_publisher.publish(msg)
        # Reduced logging

    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()

def main():
    rclpy.init()
    node = EnhancedRoadSafetyCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down enhanced road safety camera...")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
