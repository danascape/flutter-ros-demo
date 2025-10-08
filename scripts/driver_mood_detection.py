#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import base64
import numpy as np
import json
import time
from threading import Lock
import random

# Try to import emotion detection libraries
try:
    from fer import FER
    EMOTION_DETECTION_AVAILABLE = True
    print("FER (Facial Emotion Recognition) available")
except ImportError:
    EMOTION_DETECTION_AVAILABLE = False
    print("FER not available, using mock emotion detection")

try:
    import dlib
    DLIB_AVAILABLE = True
    print("dlib available for face detection")
except ImportError:
    DLIB_AVAILABLE = False
    print("dlib not available")

class DriverMoodDetectionNode(Node):
    def __init__(self):
        super().__init__('driver_mood_detection_node')

        # Publishers
        self.detection_publisher = self.create_publisher(String, '/driver_detections', 10)

        # Subscribers
        self.image_subscriber = self.create_subscription(
            String, '/camera_images', self.process_camera_image, 10)
        self.mode_subscriber = self.create_subscription(
            String, '/detection_mode', self.handle_mode_change, 10)

        # Detection mode state
        self.is_active = False  # Only process when in driver mode
        self.frame_lock = Lock()
        self.current_frame = None

        # Initialize emotion detection
        self.emotion_detector = None
        self.face_detector = None
        self.emotion_available = EMOTION_DETECTION_AVAILABLE
        self.dlib_available = DLIB_AVAILABLE

        if self.emotion_available:
            try:
                self.emotion_detector = FER(mtcnn=True)
                self.get_logger().info("FER emotion detector initialized")
            except Exception as e:
                self.get_logger().warn(f"Failed to initialize FER: {e}")
                self.emotion_available = False

        if self.dlib_available:
            try:
                self.face_detector = dlib.get_frontal_face_detector()
                self.get_logger().info("dlib face detector initialized")
            except Exception as e:
                self.get_logger().warn(f"Failed to initialize dlib: {e}")
                self.dlib_available = False

        # Driver monitoring parameters
        self.mood_history = []
        self.max_history = 20
        self.detection_interval = 0.5  # Process every 500ms
        self.last_detection_time = time.time()

        # Driver safety thresholds
        self.fatigue_threshold = 0.6
        self.alertness_threshold = 0.4
        self.emotion_confidence_threshold = 0.3

        # Detection timer
        self.detection_timer = self.create_timer(0.5, self.process_driver_detection)

        self.get_logger().info("Driver Mood Detection Node started")
        self.get_logger().info("Waiting for 'driver' mode activation...")

    def handle_mode_change(self, msg):
        """Handle detection mode changes from Flutter app"""
        try:
            mode_data = json.loads(msg.data)
            new_mode = mode_data.get('mode', 'front')

            if new_mode == 'driver':
                self.is_active = True
                self.get_logger().info("🚗 Driver detection mode ACTIVATED")
                self.publish_system_message("Driver monitoring activated")
                self.publish_system_message("Analyzing: emotions, fatigue, alertness")
            else:
                self.is_active = False
                self.get_logger().info("Driver detection mode DEACTIVATED")

        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid mode message: {msg.data}")

    def process_camera_image(self, msg):
        """Receive and store camera images for processing"""
        if not self.is_active:
            return

        try:
            # Parse image message: IMG_DATA:format:width:height:base64_data
            parts = msg.data.split(':')
            if len(parts) >= 5 and parts[0] == 'IMG_DATA':
                image_format = parts[1]
                width = int(parts[2])
                height = int(parts[3])
                base64_data = parts[4]

                # Decode base64 image
                image_bytes = base64.b64decode(base64_data)
                nparr = np.frombuffer(image_bytes, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                if frame is not None:
                    with self.frame_lock:
                        self.current_frame = frame.copy()

        except Exception as e:
            self.get_logger().error(f"Error processing camera image: {e}")

    def process_driver_detection(self):
        """Main driver detection processing"""
        if not self.is_active:
            return

        with self.frame_lock:
            if self.current_frame is None:
                return
            frame = self.current_frame.copy()

        # Run detection based on available libraries
        if self.emotion_available and self.emotion_detector is not None:
            self.detect_real_emotions(frame)
        else:
            self.detect_mock_emotions(frame)

    def detect_real_emotions(self, frame):
        """Real emotion detection using FER"""
        try:
            # Detect emotions
            emotions_result = self.emotion_detector.detect_emotions(frame)

            if not emotions_result:
                # No face detected
                self.publish_driver_detection({
                    'type': 'no_face_detected',
                    'message': 'No driver face detected',
                    'timestamp': time.time()
                })
                return

            # Process the first detected face
            for emotion_data in emotions_result[:1]:  # Take first face only
                emotions = emotion_data['emotions']
                box = emotion_data['box']

                # Find dominant emotion
                dominant_emotion = max(emotions, key=emotions.get)
                emotion_confidence = emotions[dominant_emotion]

                # Only process if confidence is high enough
                if emotion_confidence < self.emotion_confidence_threshold:
                    continue

                # Analyze driver state
                driver_analysis = self.analyze_driver_state(emotions)

                # Create detection result
                detection_result = {
                    'type': 'driver_emotion',
                    'dominant_emotion': dominant_emotion,
                    'confidence': emotion_confidence,
                    'all_emotions': emotions,
                    'alertness_level': driver_analysis['alertness'],
                    'fatigue_level': driver_analysis['fatigue'],
                    'mood_score': driver_analysis['mood_score'],
                    'safety_status': driver_analysis['safety_status'],
                    'bbox': [box[0], box[1], box[2], box[3]],
                    'timestamp': time.time()
                }

                # Store in history
                self.mood_history.append(detection_result)
                if len(self.mood_history) > self.max_history:
                    self.mood_history.pop(0)

                # Publish detection
                self.publish_driver_detection(detection_result)

                # Check for safety alerts
                self.check_safety_alerts(driver_analysis)

        except Exception as e:
            self.get_logger().error(f"Real emotion detection error: {e}")
            self.detect_mock_emotions(frame)

    def detect_mock_emotions(self, frame):
        """Mock emotion detection for testing"""
        emotions = {
            'angry': random.uniform(0.0, 0.2),
            'disgust': random.uniform(0.0, 0.1),
            'fear': random.uniform(0.0, 0.15),
            'happy': random.uniform(0.1, 0.7),
            'sad': random.uniform(0.0, 0.3),
            'surprise': random.uniform(0.0, 0.2),
            'neutral': random.uniform(0.2, 0.8)
        }

        # Simulate realistic driver emotions (mostly neutral/happy for safe driving)
        if random.random() < 0.1:  # 10% chance of fatigue/distraction
            emotions['sad'] = random.uniform(0.4, 0.8)
            emotions['neutral'] = random.uniform(0.1, 0.3)
            emotions['happy'] = random.uniform(0.0, 0.2)

        dominant_emotion = max(emotions, key=emotions.get)
        emotion_confidence = emotions[dominant_emotion]

        # Analyze driver state
        driver_analysis = self.analyze_driver_state(emotions)

        # Create detection result
        detection_result = {
            'type': 'driver_emotion',
            'dominant_emotion': dominant_emotion,
            'confidence': emotion_confidence,
            'all_emotions': emotions,
            'alertness_level': driver_analysis['alertness'],
            'fatigue_level': driver_analysis['fatigue'],
            'mood_score': driver_analysis['mood_score'],
            'safety_status': driver_analysis['safety_status'],
            'bbox': [100, 50, 200, 250],  # Mock bounding box
            'timestamp': time.time(),
            'mock': True
        }

        # Store in history
        self.mood_history.append(detection_result)
        if len(self.mood_history) > self.max_history:
            self.mood_history.pop(0)

        # Publish detection
        self.publish_driver_detection(detection_result)

        # Check for safety alerts
        self.check_safety_alerts(driver_analysis)

    def analyze_driver_state(self, emotions):
        """Analyze driver state from emotion data"""
        # Calculate alertness score (0-1, higher is more alert)
        alertness_score = (
            emotions.get('happy', 0) * 0.8 +
            emotions.get('surprise', 0) * 0.6 +
            emotions.get('neutral', 0) * 0.5 -
            emotions.get('sad', 0) * 0.7 -
            emotions.get('fear', 0) * 0.4 -
            emotions.get('angry', 0) * 0.3
        )
        alertness_score = max(0, min(1, alertness_score))

        # Calculate fatigue score (0-1, higher is more fatigued)
        fatigue_score = (
            emotions.get('sad', 0) * 0.8 +
            emotions.get('neutral', 0) * 0.4 +
            emotions.get('disgust', 0) * 0.3 +
            (1 - emotions.get('happy', 0)) * 0.2
        )
        fatigue_score = max(0, min(1, fatigue_score))

        # Calculate overall mood score
        mood_score = (
            emotions.get('happy', 0) * 1.0 +
            emotions.get('neutral', 0) * 0.6 +
            emotions.get('surprise', 0) * 0.4 -
            emotions.get('angry', 0) * 0.8 -
            emotions.get('sad', 0) * 0.6 -
            emotions.get('fear', 0) * 0.4
        )
        mood_score = max(0, min(1, mood_score))

        # Determine levels
        if alertness_score > 0.7:
            alertness_level = 'high'
        elif alertness_score > 0.4:
            alertness_level = 'medium'
        else:
            alertness_level = 'low'

        if fatigue_score > 0.6:
            fatigue_level = 'high'
        elif fatigue_score > 0.3:
            fatigue_level = 'medium'
        else:
            fatigue_level = 'low'

        # Determine safety status
        if fatigue_score > 0.7 or alertness_score < 0.3:
            safety_status = 'critical'
        elif fatigue_score > 0.5 or alertness_score < 0.5:
            safety_status = 'warning'
        elif fatigue_score > 0.3 or alertness_score < 0.7:
            safety_status = 'caution'
        else:
            safety_status = 'safe'

        return {
            'alertness': alertness_level,
            'fatigue': fatigue_level,
            'mood_score': mood_score,
            'safety_status': safety_status,
            'raw_scores': {
                'alertness_score': alertness_score,
                'fatigue_score': fatigue_score
            }
        }

    def check_safety_alerts(self, analysis):
        """Check for safety alerts and publish warnings"""
        safety_status = analysis['safety_status']

        if safety_status == 'critical':
            self.publish_safety_alert({
                'level': 'critical',
                'message': '🚨 CRITICAL: Driver appears severely fatigued or distracted!',
                'recommendation': 'Pull over safely and take a break immediately'
            })
        elif safety_status == 'warning':
            self.publish_safety_alert({
                'level': 'warning',
                'message': '⚠️ WARNING: Signs of driver fatigue detected',
                'recommendation': 'Consider taking a break soon'
            })
        elif safety_status == 'caution':
            self.publish_safety_alert({
                'level': 'caution',
                'message': '⚡ CAUTION: Monitor driver alertness',
                'recommendation': 'Stay aware of fatigue levels'
            })

    def publish_driver_detection(self, detection_data):
        """Publish driver detection result"""
        # Format: DET_DRIVER:type|emotion|confidence|alertness|fatigue|safety_status
        formatted_data = (
            f"{detection_data['type']}|"
            f"{detection_data.get('dominant_emotion', 'unknown')}|"
            f"{detection_data.get('confidence', 0):.2f}|"
            f"{detection_data['alertness_level']}|"
            f"{detection_data['fatigue_level']}|"
            f"{detection_data['safety_status']}"
        )

        msg = String()
        msg.data = f"DET_DRIVER:{formatted_data}"
        self.detection_publisher.publish(msg)

        # Log important detections
        if detection_data['safety_status'] in ['critical', 'warning']:
            emotion = detection_data.get('dominant_emotion', 'unknown')
            confidence = detection_data.get('confidence', 0)
            self.get_logger().warn(
                f"DRIVER ALERT: {detection_data['safety_status'].upper()} - "
                f"{emotion} ({confidence:.1%}), "
                f"Fatigue: {detection_data['fatigue_level']}"
            )

    def publish_safety_alert(self, alert_data):
        """Publish safety alert"""
        msg = String()
        msg.data = f"SAFETY_ALERT:{alert_data['level']}|{alert_data['message']}|{alert_data['recommendation']}"
        self.detection_publisher.publish(msg)

        self.get_logger().warn(f"SAFETY: {alert_data['message']}")

    def publish_system_message(self, message):
        """Publish system message for UI"""
        msg = String()
        msg.data = f"SYSTEM:{message}"
        self.detection_publisher.publish(msg)

def main():
    rclpy.init()
    node = DriverMoodDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down driver mood detection...")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()