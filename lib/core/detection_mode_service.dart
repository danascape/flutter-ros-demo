import 'dart:async';
import 'dart:convert';
import 'package:rcldart/rcldart.dart' as rcldart;
import 'package:rcldart/src/node.dart';
import 'package:rcldart/src/publisher.dart' as publish;
import 'package:rcldart/src/subscriber.dart' as rclsubscriber;
import 'package:std_msgs/std_msgs.dart';
import '../utils/message_utils.dart';

enum DetectionMode { driver, front }

class ModeStatus {
  final DetectionMode currentMode;
  final List<DetectionMode> availableModes;
  final String status;
  final String? error;
  final DateTime timestamp;

  ModeStatus({
    required this.currentMode,
    required this.availableModes,
    required this.status,
    this.error,
    required this.timestamp,
  });

  factory ModeStatus.fromJson(Map<String, dynamic> json) {
    return ModeStatus(
      currentMode: json['current_mode'] == 'driver'
          ? DetectionMode.driver
          : DetectionMode.front,
      availableModes: (json['available_modes'] as List<dynamic>?)
              ?.map((mode) => mode == 'driver'
                  ? DetectionMode.driver
                  : DetectionMode.front)
              .toList() ??
          [DetectionMode.driver, DetectionMode.front],
      status: json['status'] ?? 'unknown',
      error: json['error'],
      timestamp: DateTime.now(),
    );
  }
}

class DetectionModeService {
  Node? _node;
  publish.Publisher? _modeCommandPublisher;
  rclsubscriber.Subscriber? _modeStatusSubscriber;
  Timer? _pollingTimer;

  late StreamController<ModeStatus> _modeStatusStreamController;

  DetectionMode _currentMode = DetectionMode.front;

  Stream<ModeStatus> get modeStatusStream => _modeStatusStreamController.stream;
  DetectionMode get currentMode => _currentMode;

  DetectionModeService() {
    _modeStatusStreamController = StreamController<ModeStatus>.broadcast();
  }

  Future<void> initialize() async {
    try {
      _node = rcldart.RclDart().createNode("flutter_mode_controller", "flutter_app");

      // Publisher for mode commands
      _modeCommandPublisher = _node!.createPublisher<StdMsgsString>(
        topic_name: '/mode_command',
        messageType: StdMsgsString(""),
      );

      // Subscriber for mode status updates
      _modeStatusSubscriber = _node!.createSubscriber<StdMsgsString>(
        topic_name: '/mode_status',
        messageType: StdMsgsString(""),
        callback: (msg) {
          _processModeStatusMessage(msg);
        },
      );

      _modeStatusSubscriber!.subscribe();
      await Future.delayed(Duration(milliseconds: 500));
      _startBackgroundPolling();

      print('DetectionModeService initialized successfully');
    } catch (e) {
      throw DetectionModeServiceException("Failed to initialize mode service: $e");
    }
  }

  void _processModeStatusMessage(StdMsgsString msg) {
    try {
      final jsonData = json.decode(msg.value);
      final modeStatus = ModeStatus.fromJson(jsonData);
      _currentMode = modeStatus.currentMode;
      _modeStatusStreamController.add(modeStatus);
      print('Mode status updated: ${modeStatus.currentMode}');
    } catch (e) {
      print('Error processing mode status message: $e');
    }
  }

  void _startBackgroundPolling() {
    _pollingTimer = Timer.periodic(Duration(milliseconds: 100), (timer) {
      try {
        if (_modeStatusSubscriber != null) {
          _modeStatusSubscriber!.take();
        }
      } catch (e) {
        // Ignore polling errors
      }
    });
  }

  Future<void> switchMode(DetectionMode mode) async {
    if (_modeCommandPublisher == null) {
      throw DetectionModeServiceException("Mode command publisher not initialized");
    }

    try {
      final modeCommand = {
        'mode': mode == DetectionMode.driver ? 'driver' : 'front',
        'timestamp': DateTime.now().millisecondsSinceEpoch / 1000.0,
      };

      final commandJson = json.encode(modeCommand);
      final message = MessageUtils.createFixedStdMsgsString(commandJson);

      _modeCommandPublisher!.publish(message);
      _currentMode = mode;

      print('Mode switch command sent: ${mode == DetectionMode.driver ? "driver" : "front"}');
    } catch (e) {
      throw DetectionModeServiceException("Failed to switch mode: $e");
    }
  }

  bool get isInitialized => _modeCommandPublisher != null && _modeStatusSubscriber != null;

  void dispose() {
    _pollingTimer?.cancel();
    _modeStatusStreamController.close();
  }
}

class DetectionModeServiceException implements Exception {
  final String message;
  DetectionModeServiceException(this.message);

  @override
  String toString() => 'DetectionModeServiceException: $message';
}
