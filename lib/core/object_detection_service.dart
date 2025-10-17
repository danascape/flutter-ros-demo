import 'dart:async';
import 'package:rcldart/rcldart.dart' as rcldart;
import 'package:rcldart/src/node.dart';
import 'package:rcldart/src/subscriber.dart' as rclsubscriber;
import 'package:std_msgs/std_msgs.dart';
import '../utils/simple_messages.dart';

// Use the SimpleDetection class from simple_messages.dart as DetectionResult
typedef DetectionResult = SimpleDetection;
typedef BoundingBox = SimpleBoundingBox;

class ObjectDetectionService {
  Node? _node;
  rclsubscriber.Subscriber? _detectionSubscription;
  rclsubscriber.Subscriber? _driverDetectionSubscription;
  Timer? _pollingTimer;
  late StreamController<DetectionResult> _detectionStreamController;
  late StreamController<DriverDetection> _driverDetectionStreamController;
  late StreamController<SafetyAlert> _safetyAlertStreamController;

  Stream<DetectionResult> get detectionStream => _detectionStreamController.stream;
  Stream<DriverDetection> get driverDetectionStream => _driverDetectionStreamController.stream;
  Stream<SafetyAlert> get safetyAlertStream => _safetyAlertStreamController.stream;

  ObjectDetectionService() {
    _detectionStreamController = StreamController<DetectionResult>.broadcast();
    _driverDetectionStreamController = StreamController<DriverDetection>.broadcast();
    _safetyAlertStreamController = StreamController<SafetyAlert>.broadcast();
  }

  Future<void> initialize() async {
    try {
      _node = rcldart.RclDart().createNode("flutter_detection_node", "flutter_app");

      // Subscribe to front object detections
      _detectionSubscription = _node!.createSubscriber<StdMsgsString>(
        topic_name: "/object_detections",
        messageType: StdMsgsString(""),
        callback: (msg) {
          print("Object detection callback triggered!");
          _processDetectionMessage(msg);
        },
      );

      // Subscribe to driver detections (mood, emotion, fatigue)
      _driverDetectionSubscription = _node!.createSubscriber<StdMsgsString>(
        topic_name: "/driver_detections",
        messageType: StdMsgsString(""),
        callback: (msg) {
          print("Driver detection callback triggered!");
          _processDriverDetectionMessage(msg);
        },
      );

      _detectionSubscription!.subscribe();
      _driverDetectionSubscription!.subscribe();

      await Future.delayed(Duration(milliseconds: 500));
      _startBackgroundPolling();

      print('ObjectDetectionService initialized with both object and driver detection');
    } catch (e) {
      throw ObjectDetectionServiceException("Failed to initialize detection service: $e");
    }
  }

  void _processDetectionMessage(StdMsgsString msg) {
    try {
      print("Raw detection message received: '${msg.value}'");
      final detection = MessageConverter.parseDetectionMessage(msg.value);
      print("Parsed detection: ${detection.className} confidence=${detection.confidence}");
      _detectionStreamController.add(detection);
      print("Detection added to stream");
    } catch (e) {
      print("Error processing detection message: $e");
    }
  }

  void _processDriverDetectionMessage(StdMsgsString msg) {
    try {
      final messageValue = msg.value;
      print("Raw driver detection message received: '$messageValue'");

      // Handle different message types from driver detection node
      if (messageValue.startsWith('DET_DRIVER:')) {
        final driverDetection = MessageConverter.parseDriverDetectionMessage(messageValue);
        print("Parsed driver detection: ${driverDetection.dominantEmotion} (${driverDetection.safetyStatus})");
        _driverDetectionStreamController.add(driverDetection);
        print("Driver detection added to stream");
      } else if (messageValue.startsWith('SAFETY_ALERT:')) {
        final safetyAlert = MessageConverter.parseSafetyAlertMessage(messageValue);
        print("Parsed safety alert: ${safetyAlert.level} - ${safetyAlert.message}");
        _safetyAlertStreamController.add(safetyAlert);
        print("Safety alert added to stream");
      } else if (messageValue.startsWith('SYSTEM:')) {
        // System messages - can be logged or displayed
        final systemMessage = messageValue.substring(7);
        print("System message: $systemMessage");
      }
    } catch (e) {
      print("Error processing driver detection message: $e");
    }
  }

  void _startBackgroundPolling() {
    _pollingTimer = Timer.periodic(Duration(milliseconds: 100), (timer) {
      try {
        if (_detectionSubscription != null) {
          _detectionSubscription!.take();
        }
        if (_driverDetectionSubscription != null) {
          _driverDetectionSubscription!.take();
        }
      } catch (e) {
        // Ignore polling errors
      }
    });
  }

  bool get isInitialized => _detectionSubscription != null && _driverDetectionSubscription != null;

  void dispose() {
    _pollingTimer?.cancel();
    _detectionStreamController.close();
    _driverDetectionStreamController.close();
    _safetyAlertStreamController.close();
  }
}

class ObjectDetectionServiceException implements Exception {
  final String message;
  ObjectDetectionServiceException(this.message);

  @override
  String toString() => 'ObjectDetectionServiceException: $message';
}