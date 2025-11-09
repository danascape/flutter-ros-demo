import 'dart:async';
import 'package:rcldart/rcldart.dart' as rcldart;
import 'package:rcldart/src/node.dart';
import 'package:rcldart/src/subscriber.dart' as rclsubscriber;
import 'package:std_msgs/std_msgs.dart';
import '../utils/simple_messages.dart';

class CameraService {
  Node? _node;
  rclsubscriber.Subscriber? _imageSubscription;
  Timer? _pollingTimer;
  late StreamController<SimpleImageData> _imageStreamController;

  Stream<SimpleImageData> get imageStream => _imageStreamController.stream;

  CameraService() {
    _imageStreamController = StreamController<SimpleImageData>.broadcast();
  }

  Future<void> initialize() async {
    try {
      _node = rcldart.RclDart().createNode("flutter_camera_node", "flutter_app");

      _imageSubscription = _node!.createSubscriber<StdMsgsString>(
        topic_name: "/camera_images",
        messageType: StdMsgsString(""),
        callback: (msg) {
          _processImageMessage(msg);
        },
      );

      _imageSubscription!.subscribe();
      await Future.delayed(Duration(milliseconds: 500));
      _startBackgroundPolling();
    } catch (e) {
      throw CameraServiceException("Failed to initialize camera service: $e");
    }
  }

  void _processImageMessage(StdMsgsString msg) {
    try {
      final imageData = MessageConverter.parseImageMessage(msg.value);
      _imageStreamController.add(imageData);
    } catch (e) {
      print("Error processing image message: $e");
    }
  }

  void _startBackgroundPolling() {
    _pollingTimer = Timer.periodic(Duration(milliseconds: 100), (timer) {
      try {
        if (_imageSubscription != null) {
          _imageSubscription!.take();
        }
      } catch (e) {
        // Ignore polling errors
      }
    });
  }

  bool get isInitialized => _imageSubscription != null;

  void dispose() {
    _pollingTimer?.cancel();
    _imageStreamController.close();
  }
}

class CameraServiceException implements Exception {
  final String message;
  CameraServiceException(this.message);

  @override
  String toString() => 'CameraServiceException: $message';
}