import 'package:flutter/material.dart';
import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';
import '../core/camera_service.dart';
import '../core/object_detection_service.dart';
import '../core/detection_mode_service.dart';
import '../utils/detection_utils.dart';
import '../utils/simple_messages.dart';

enum CameraModel { driver, front }

class ObjectDetectionPage extends StatefulWidget {
  const ObjectDetectionPage({super.key});

  @override
  State<ObjectDetectionPage> createState() => _ObjectDetectionPageState();
}

class _ObjectDetectionPageState extends State<ObjectDetectionPage> {
  final CameraService _cameraService = CameraService();
  final ObjectDetectionService _detectionService = ObjectDetectionService();
  final DetectionModeService _modeService = DetectionModeService();

  List<String> _detectionResults = [];
  List<String> _systemMessages = [];
  SimpleImageData? _currentImageData;
  MemoryImage? _currentMemoryImage;
  MemoryImage? _previousMemoryImage;
  bool _isConnected = false;
  bool _showBrakeWarning = false;
  String _warningMessage = '';
  String _warningSource = '';
  int _frameCounter = 0;
  DateTime _lastFrameTime = DateTime.now();

  // Camera detection model selection
  CameraModel _selectedCameraModel = CameraModel.driver;

  // Driver mood data
  DriverDetection? _currentDriverMood;
  String _driverMoodDisplay = 'No data';

  late StreamSubscription<SimpleImageData> _imageSubscription;
  late StreamSubscription<DetectionResult> _detectionSubscription;
  late StreamSubscription<DriverDetection> _driverDetectionSubscription;
  late StreamSubscription<SafetyAlert> _safetyAlertSubscription;

  final ScrollController _detectionScrollController = ScrollController();
  final ScrollController _systemMessagesScrollController = ScrollController();

  @override
  void initState() {
    super.initState();
    _initializeServices();
  }

  void _initializeServices() async {
    try {
      await _cameraService.initialize();
      await _detectionService.initialize();
      await _modeService.initialize();

      _setupImageListener();
      _setupDetectionListener();
      _setupDriverDetectionListener();
      _setupSafetyAlertListener();

      // Set initial mode to driver (person detection)
      await _modeService.switchMode(DetectionMode.driver);

      if (mounted) {
        setState(() {
          _isConnected = true;
          _systemMessages.add("Camera and detection services initialized");
          _systemMessages.add("Mode: Driver Detection - Person Detection (default)");
        });
        _scrollSystemMessagesToBottom();
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          _systemMessages.add("Error initializing services: $e");
        });
        _scrollSystemMessagesToBottom();
      }
    }
  }

  void _setupImageListener() {
    _imageSubscription = _cameraService.imageStream.listen((imageData) {
      if (mounted && imageData.hasImageData) {
        // Throttle frame updates to reduce blinking
        final now = DateTime.now();
        if (now.difference(_lastFrameTime).inMilliseconds < 100) {
          return; // Skip frame if too soon
        }
        _lastFrameTime = now;

        try {
          final newImageBytes = base64Decode(imageData.base64Data!);
          final newMemoryImage = MemoryImage(newImageBytes);

          // Only update if we have a different frame
          _frameCounter++;

          setState(() {
            _previousMemoryImage = _currentMemoryImage;
            _currentMemoryImage = newMemoryImage;
            _currentImageData = imageData;
          });
        } catch (e) {
          print('Error processing image: $e');
        }
      }
    });
  }

  void _setupDetectionListener() {
    _detectionSubscription = _detectionService.detectionStream.listen((result) {
      print("UI received detection result: ${result.className}");
      if (mounted) {
        setState(() {
          _detectionResults.add(DetectionUtils.formatDetectionResult(result));

          // Check for brake warning conditions (front detection)
          _checkBrakeWarning(result);

          if (_detectionResults.length > 50) {
            _detectionResults.removeAt(0);
          }
          if (_systemMessages.length > 20) {
            _systemMessages.removeAt(0);
          }
        });
        _scrollDetectionToBottom();
        _scrollSystemMessagesToBottom();
      }
    });
  }

  void _setupDriverDetectionListener() {
    _driverDetectionSubscription = _detectionService.driverDetectionStream.listen((driverDetection) {
      print("UI received driver detection: ${driverDetection.dominantEmotion} (${driverDetection.safetyStatus})");
      if (mounted) {
        setState(() {
          _currentDriverMood = driverDetection;
          _driverMoodDisplay = '${driverDetection.emotionEmoji} ${driverDetection.dominantEmotion} '
              '(${(driverDetection.confidence * 100).toStringAsFixed(0)}%) - '
              '${driverDetection.statusEmoji} ${driverDetection.safetyStatus.toUpperCase()}';

          _detectionResults.add(
            '${driverDetection.emotionEmoji} ${driverDetection.dominantEmotion} | '
            'Alertness: ${driverDetection.alertnessLevel} | '
            'Fatigue: ${driverDetection.fatigueLevel} | '
            '${driverDetection.statusEmoji} ${driverDetection.safetyStatus}'
          );

          // Check for brake warning if driver is sleepy/fatigued
          _checkDriverBrakeWarning(driverDetection);

          if (_detectionResults.length > 50) {
            _detectionResults.removeAt(0);
          }
        });
        _scrollDetectionToBottom();
      }
    });
  }

  void _setupSafetyAlertListener() {
    _safetyAlertSubscription = _detectionService.safetyAlertStream.listen((alert) {
      print("UI received safety alert: ${alert.level} - ${alert.message}");
      if (mounted) {
        setState(() {
          _systemMessages.add('${alert.level.toUpperCase()}: ${alert.message}');

          // Show brake warning for critical alerts
          if (alert.isCritical) {
            _showBrakeWarning = true;
            _warningMessage = alert.message;
            _warningSource = 'driver';

            // Auto-hide warning after 5 seconds for safety alerts
            Future.delayed(Duration(seconds: 5), () {
              if (mounted) {
                setState(() {
                  _showBrakeWarning = false;
                });
              }
            });
          }

          if (_systemMessages.length > 20) {
            _systemMessages.removeAt(0);
          }
        });
        _scrollSystemMessagesToBottom();
      }
    });
  }

  void _checkBrakeWarning(DetectionResult result) {
    if (result.isCritical) {
      setState(() {
        _showBrakeWarning = true;
        _warningMessage = '🚨 BRAKE! ${result.className.toUpperCase()} AT ${result.distance ?? 'CLOSE RANGE'}';
        _warningSource = 'front';
      });

      // Auto-hide warning after 3 seconds
      Future.delayed(Duration(seconds: 3), () {
        if (mounted) {
          setState(() {
            _showBrakeWarning = false;
          });
        }
      });
    } else if (result.isWarning) {
      // Brief warning flash
      setState(() {
        _showBrakeWarning = true;
        _warningMessage = '⚠️ CAUTION: ${result.className} at ${result.distance ?? 'near'}';
        _warningSource = 'front';
      });

      Future.delayed(Duration(milliseconds: 1500), () {
        if (mounted) {
          setState(() {
            _showBrakeWarning = false;
          });
        }
      });
    }
  }

  void _checkDriverBrakeWarning(DriverDetection driverDetection) {
    // Issue brake warning if person is sleepy or critically fatigued
    if (driverDetection.needsBrakeWarning) {
      setState(() {
        _showBrakeWarning = true;
        _warningMessage = '🚨 BRAKE WARNING: PERSON SLEEPY/FATIGUED!\n'
            'Emotion: ${driverDetection.emotionEmoji} ${driverDetection.dominantEmotion}\n'
            'Fatigue: ${driverDetection.fatigueLevel.toUpperCase()}\n'
            'PULL OVER IMMEDIATELY!';
        _warningSource = 'mood';
      });

      // Keep warning visible longer for fatigue detection
      Future.delayed(Duration(seconds: 5), () {
        if (mounted) {
          setState(() {
            _showBrakeWarning = false;
          });
        }
      });
    }
  }

  @override
  void dispose() {
    _imageSubscription.cancel();
    _detectionSubscription.cancel();
    _driverDetectionSubscription.cancel();
    _safetyAlertSubscription.cancel();
    _detectionScrollController.dispose();
    _systemMessagesScrollController.dispose();
    _cameraService.dispose();
    _detectionService.dispose();
    _modeService.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: const Text('Road Safety Detection'),
        actions: [
          Icon(
            _isConnected ? Icons.wifi : Icons.wifi_off,
            color: _isConnected ? Colors.green : Colors.red,
          ),
          const SizedBox(width: 16),
        ],
      ),
      body: Stack(
        children: [
          Padding(
            padding: const EdgeInsets.all(16.0),
            child: Column(
              children: [
                Expanded(
                  child: Row(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      // Left column - Camera stream
                      Expanded(
                        flex: 2,
                        child: _buildCameraStreamCard(context),
                      ),
                      const SizedBox(width: 16),
                      // Right column - Controls and messages
                      Expanded(
                        flex: 3,
                        child: Column(
                          children: [
                            _buildStreamControlCard(context),
                            const SizedBox(height: 16),
                            // Driver mood display (only show when in FRONT mode - swapped)
                            if (_selectedCameraModel == CameraModel.front)
                              _buildDriverMoodCard(context),
                            if (_selectedCameraModel == CameraModel.front)
                              const SizedBox(height: 16),
                            Expanded(
                              child: Row(
                                children: [
                                  Expanded(child: _buildDetectionResultsCard(context)),
                                  const SizedBox(width: 16),
                                  Expanded(child: _buildSystemMessagesCard(context)),
                                ],
                              ),
                            ),
                          ],
                        ),
                      ),
                    ],
                  ),
                ),
              ],
            ),
          ),
          // Brake Warning Overlay
          if (_showBrakeWarning) _buildBrakeWarningOverlay(context),
        ],
      ),
    );
  }

  Widget _buildCameraStreamCard(BuildContext context) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Camera Stream',
              style: Theme.of(context).textTheme.titleMedium,
            ),
            const SizedBox(height: 8),
            Expanded(
              child: Container(
                width: double.infinity,
                decoration: BoxDecoration(
                  border: Border.all(color: Colors.grey),
                  borderRadius: BorderRadius.circular(8),
                ),
                child: ClipRRect(
                  borderRadius: BorderRadius.circular(8),
                  child: RepaintBoundary(
                    child: _currentMemoryImage != null
                        ? _buildStableImageDisplay()
                        : const Center(
                            child: Text(
                              'No camera stream available',
                              style: TextStyle(color: Colors.grey),
                            ),
                          ),
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildDriverMoodCard(BuildContext context) {
    final driver = _currentDriverMood;
    Color statusColor;

    if (driver == null) {
      statusColor = Colors.grey;
    } else if (driver.safetyStatus == 'critical') {
      statusColor = Colors.red;
    } else if (driver.safetyStatus == 'warning') {
      statusColor = Colors.orange;
    } else if (driver.safetyStatus == 'caution') {
      statusColor = Colors.yellow.shade700;
    } else {
      statusColor = Colors.green;
    }

    return Card(
      color: statusColor.withOpacity(0.1),
      elevation: 4,
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(Icons.person, color: statusColor, size: 24),
                const SizedBox(width: 8),
                Text(
                  'Mood Monitor (Front Detection)',
                  style: Theme.of(context).textTheme.titleMedium?.copyWith(
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ],
            ),
            const SizedBox(height: 12),
            if (driver == null)
              Text(
                'Waiting for mood detection data...',
                style: TextStyle(color: Colors.grey, fontSize: 14),
              )
            else
              Column(
                children: [
                  Row(
                    mainAxisAlignment: MainAxisAlignment.spaceAround,
                    children: [
                      _buildMoodMetric(
                        context,
                        'Emotion',
                        '${driver.emotionEmoji} ${driver.dominantEmotion}',
                        '${(driver.confidence * 100).toStringAsFixed(0)}%',
                        Colors.blue,
                      ),
                      _buildMoodMetric(
                        context,
                        'Alertness',
                        driver.alertnessLevel.toUpperCase(),
                        driver.alertnessLevel == 'high' ? '✓' : '!',
                        driver.alertnessLevel == 'high' ? Colors.green : Colors.orange,
                      ),
                      _buildMoodMetric(
                        context,
                        'Fatigue',
                        driver.fatigueLevel.toUpperCase(),
                        driver.fatigueLevel == 'high' ? '⚠' : '✓',
                        driver.fatigueLevel == 'high' ? Colors.red : Colors.green,
                      ),
                      _buildMoodMetric(
                        context,
                        'Status',
                        driver.safetyStatus.toUpperCase(),
                        driver.statusEmoji,
                        statusColor,
                      ),
                    ],
                  ),
                  if (driver.needsBrakeWarning) ...[
                    const SizedBox(height: 12),
                    Container(
                      width: double.infinity,
                      padding: const EdgeInsets.all(12),
                      decoration: BoxDecoration(
                        color: Colors.red.withOpacity(0.2),
                        borderRadius: BorderRadius.circular(8),
                        border: Border.all(color: Colors.red, width: 2),
                      ),
                      child: Row(
                        children: [
                          Icon(Icons.warning, color: Colors.red, size: 24),
                          const SizedBox(width: 8),
                          Expanded(
                            child: Text(
                              'FATIGUE DETECTED - BRAKE WARNING ACTIVE',
                              style: TextStyle(
                                color: Colors.red,
                                fontWeight: FontWeight.bold,
                                fontSize: 12,
                              ),
                            ),
                          ),
                        ],
                      ),
                    ),
                  ],
                ],
              ),
          ],
        ),
      ),
    );
  }

  Widget _buildMoodMetric(BuildContext context, String label, String value, String indicator, Color color) {
    return Column(
      children: [
        Text(
          label,
          style: TextStyle(
            fontSize: 12,
            color: Colors.grey[600],
            fontWeight: FontWeight.w500,
          ),
        ),
        const SizedBox(height: 4),
        Container(
          padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
          decoration: BoxDecoration(
            color: color.withOpacity(0.1),
            borderRadius: BorderRadius.circular(8),
            border: Border.all(color: color, width: 2),
          ),
          child: Column(
            children: [
              Text(
                indicator,
                style: TextStyle(
                  fontSize: 20,
                  fontWeight: FontWeight.bold,
                  color: color,
                ),
              ),
              const SizedBox(height: 2),
              Text(
                value,
                style: TextStyle(
                  fontSize: 11,
                  fontWeight: FontWeight.bold,
                  color: color,
                ),
                textAlign: TextAlign.center,
              ),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildStreamControlCard(BuildContext context) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Detection Model',
              style: Theme.of(context).textTheme.titleMedium,
            ),
            const SizedBox(height: 12),
            Column(
              children: [
                RadioListTile<CameraModel>(
                  title: Row(
                    children: [
                      Icon(
                        Icons.person,
                        color: _selectedCameraModel == CameraModel.driver
                            ? Theme.of(context).primaryColor
                            : Colors.grey,
                        size: 20,
                      ),
                      const SizedBox(width: 8),
                      const Text('Driver Detection'),
                    ],
                  ),
                  subtitle: const Text('Detect person/objects (driver as person)'),
                  value: CameraModel.driver,
                  groupValue: _selectedCameraModel,
                  activeColor: Theme.of(context).primaryColor,
                  onChanged: (CameraModel? value) {
                    setState(() {
                      _selectedCameraModel = value!;
                    });
                    _onCameraModelChanged(value!);
                  },
                ),
                RadioListTile<CameraModel>(
                  title: Row(
                    children: [
                      Icon(
                        Icons.mood,
                        color: _selectedCameraModel == CameraModel.front
                            ? Theme.of(context).primaryColor
                            : Colors.grey,
                        size: 20,
                      ),
                      const SizedBox(width: 8),
                      const Text('Front Detection'),
                    ],
                  ),
                  subtitle: const Text('Mood and emotion detection'),
                  value: CameraModel.front,
                  groupValue: _selectedCameraModel,
                  activeColor: Theme.of(context).primaryColor,
                  onChanged: (CameraModel? value) {
                    setState(() {
                      _selectedCameraModel = value!;
                    });
                    _onCameraModelChanged(value!);
                  },
                ),
              ],
            ),
            const SizedBox(height: 16),
            Divider(color: Colors.grey.withOpacity(0.3)),
            const SizedBox(height: 8),
            Row(
              children: [
                Text(
                  'Stream Status:',
                  style: Theme.of(context).textTheme.bodySmall,
                ),
                const SizedBox(width: 8),
                Icon(
                  _isConnected ? Icons.circle : Icons.circle_outlined,
                  color: _isConnected ? Colors.green : Colors.red,
                  size: 16,
                ),
                const SizedBox(width: 4),
                Text(
                  _isConnected ? 'Connected' : 'Disconnected',
                  style: TextStyle(
                    color: _isConnected ? Colors.green : Colors.red,
                    fontSize: 12,
                    fontWeight: FontWeight.w500,
                  ),
                ),
              ],
            ),
            if (_currentImageData != null) ...
            [
              const SizedBox(height: 8),
              Row(
                children: [
                  Container(
                    padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                    decoration: BoxDecoration(
                      color: Colors.green.withOpacity(0.1),
                      borderRadius: BorderRadius.circular(4),
                      border: Border.all(color: Colors.green),
                    ),
                    child: Text(
                      '${_currentImageData!.width}x${_currentImageData!.height}',
                      style: const TextStyle(
                        color: Colors.green,
                        fontSize: 12,
                        fontFamily: 'monospace',
                      ),
                    ),
                  ),
                  const SizedBox(width: 8),
                  Container(
                    padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                    decoration: BoxDecoration(
                      color: Colors.blue.withOpacity(0.1),
                      borderRadius: BorderRadius.circular(4),
                      border: Border.all(color: Colors.blue),
                    ),
                    child: Text(
                      _currentImageData!.format.toUpperCase(),
                      style: const TextStyle(
                        color: Colors.blue,
                        fontSize: 12,
                        fontFamily: 'monospace',
                      ),
                    ),
                  ),
                  const Spacer(),
                  Container(
                    padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                    decoration: BoxDecoration(
                      color: Colors.orange.withOpacity(0.1),
                      borderRadius: BorderRadius.circular(4),
                      border: Border.all(color: Colors.orange),
                    ),
                    child: Text(
                      _selectedCameraModel == CameraModel.driver ? 'DRIVER' : 'FRONT',
                      style: const TextStyle(
                        color: Colors.orange,
                        fontSize: 12,
                        fontFamily: 'monospace',
                        fontWeight: FontWeight.bold,
                      ),
                    ),
                  ),
                ],
              ),
            ]
          ],
        ),
      ),
    );
  }

  void _scrollDetectionToBottom() {
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (_detectionScrollController.hasClients) {
        _detectionScrollController.animateTo(
          _detectionScrollController.position.maxScrollExtent,
          duration: const Duration(milliseconds: 300),
          curve: Curves.easeOut,
        );
      }
    });
  }

  void _scrollSystemMessagesToBottom() {
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (_systemMessagesScrollController.hasClients) {
        _systemMessagesScrollController.animateTo(
          _systemMessagesScrollController.position.maxScrollExtent,
          duration: const Duration(milliseconds: 300),
          curve: Curves.easeOut,
        );
      }
    });
  }

  void _onCameraModelChanged(CameraModel model) {
    print('Camera model changed to: ${model.name}');

    setState(() {
      _systemMessages.add('Detection model switched to: ${model.name.toUpperCase()}');
      if (_systemMessages.length > 20) {
        _systemMessages.removeAt(0);
      }
    });
    _scrollSystemMessagesToBottom();

    // Switch detection model via ROS service
    _switchDetectionModel(model);
  }

  void _switchDetectionModel(CameraModel model) async {
    // Publish mode switch command to ROS2 via mode service
    try {
      final detectionMode = model == CameraModel.driver ? DetectionMode.driver : DetectionMode.front;
      await _modeService.switchMode(detectionMode);

      if (model == CameraModel.driver) {
        _switchToDriverDetection();
      } else {
        _switchToFrontDetection();
      }

      print('Mode switched to: ${model.name}');
    } catch (e) {
      setState(() {
        _systemMessages.add('Error switching detection model: $e');
      });
      _scrollSystemMessagesToBottom();
    }
  }

  void _switchToDriverDetection() {
    setState(() {
      _systemMessages.add('✓ Driver Detection Mode Active');
      _systemMessages.add('  → Monitoring: Person/object detection');
      _systemMessages.add('  → Analyzing: Driver as person/object');
      _systemMessages.add('  → Brake warning: If objects are too near');
    });
    _scrollSystemMessagesToBottom();
  }

  void _switchToFrontDetection() {
    setState(() {
      _systemMessages.add('✓ Front Detection Mode Active');
      _systemMessages.add('  → Monitoring: Mood and emotions');
      _systemMessages.add('  → Analyzing: Fatigue, alertness, emotions');
      _systemMessages.add('  → Brake warning: If person is sleepy/fatigued');
    });
    _scrollSystemMessagesToBottom();
  }

  Widget _buildDetectionResultsCard(BuildContext context) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Detection Results',
              style: Theme.of(context).textTheme.titleMedium,
            ),
            const SizedBox(height: 8),
            Expanded(
              child: ListView.builder(
                controller: _detectionScrollController,
                itemCount: _detectionResults.length,
                itemBuilder: (context, index) {
                  return Padding(
                    padding: const EdgeInsets.symmetric(vertical: 2.0),
                    child: Text(
                      _detectionResults[index],
                      style: const TextStyle(fontFamily: 'monospace', fontSize: 16),
                    ),
                  );
                },
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildSystemMessagesCard(BuildContext context) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'System Messages',
              style: Theme.of(context).textTheme.titleMedium,
            ),
            const SizedBox(height: 8),
            Expanded(
              child: ListView.builder(
                controller: _systemMessagesScrollController,
                itemCount: _systemMessages.length,
                itemBuilder: (context, index) {
                  return Padding(
                    padding: const EdgeInsets.symmetric(vertical: 2.0),
                    child: Text(
                      _systemMessages[index],
                      style: const TextStyle(fontFamily: 'monospace', fontSize: 16),
                    ),
                  );
                },
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildStableImageDisplay() {
    return Container(
      key: ValueKey('stable_image_$_frameCounter'),
      child: Image(
        image: _currentMemoryImage!,
        fit: BoxFit.contain,
        width: double.infinity,
        height: double.infinity,
        gaplessPlayback: true,
        filterQuality: FilterQuality.medium,
        frameBuilder: (context, child, frame, wasSynchronouslyLoaded) {
          // Only show the image when it's fully loaded
          if (wasSynchronouslyLoaded || frame != null) {
            return child;
          }
          // Show previous image while loading new one
          return _previousMemoryImage != null
              ? Image(
                  image: _previousMemoryImage!,
                  fit: BoxFit.contain,
                  width: double.infinity,
                  height: double.infinity,
                  gaplessPlayback: true,
                )
              : const Center(
                  child: CircularProgressIndicator(),
                );
        },
        errorBuilder: (context, error, stackTrace) {
          print('Image display error: $error');
          return const Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Icon(Icons.error, color: Colors.red),
                SizedBox(height: 8),
                Text('Image error', style: TextStyle(color: Colors.red)),
              ],
            ),
          );
        },
      ),
    );
  }

  Widget _buildImageInfo() {
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Text(
            'Image: ${_currentImageData!.data}',
            style: const TextStyle(color: Colors.grey, fontSize: 16),
          ),
          Text(
            'Format: ${_currentImageData!.format}',
            style: const TextStyle(color: Colors.grey, fontSize: 16),
          ),
          Text(
            'Resolution: ${_currentImageData!.width}x${_currentImageData!.height}',
            style: const TextStyle(color: Colors.grey, fontSize: 16),
          ),
        ],
      ),
    );
  }

  Widget _buildBrakeWarningOverlay(BuildContext context) {
    final bool isCritical = _warningMessage.startsWith('🚨');
    final bool isMoodWarning = _warningSource == 'mood';
    final bool isPersonWarning = _warningSource == 'driver';

    String alertType;
    if (isMoodWarning) {
      alertType = '🎭 MOOD ALERT';
    } else if (isPersonWarning) {
      alertType = '👤 PERSON ALERT';
    } else {
      alertType = '🛣️ ROAD ALERT';
    }

    return Positioned.fill(
      child: Container(
        color: isCritical
            ? Colors.red.withOpacity(0.95)
            : Colors.orange.withOpacity(0.85),
        child: Center(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Container(
                padding: const EdgeInsets.all(32),
                constraints: BoxConstraints(maxWidth: 600),
                decoration: BoxDecoration(
                  color: Colors.white,
                  borderRadius: BorderRadius.circular(20),
                  border: Border.all(
                    color: isCritical ? Colors.red : Colors.orange,
                    width: 6,
                  ),
                  boxShadow: [
                    BoxShadow(
                      color: Colors.black.withOpacity(0.3),
                      blurRadius: 20,
                      spreadRadius: 5,
                    ),
                  ],
                ),
                child: Column(
                  children: [
                    Icon(
                      isCritical ? Icons.warning_amber_rounded : Icons.info_outline,
                      size: 80,
                      color: isCritical ? Colors.red : Colors.orange,
                    ),
                    const SizedBox(height: 20),
                    Container(
                      padding: const EdgeInsets.all(12),
                      decoration: BoxDecoration(
                        color: (isCritical ? Colors.red : Colors.orange).withOpacity(0.1),
                        borderRadius: BorderRadius.circular(8),
                      ),
                      child: Text(
                        alertType,
                        style: TextStyle(
                          fontSize: 18,
                          fontWeight: FontWeight.bold,
                          color: isCritical ? Colors.red : Colors.orange,
                        ),
                      ),
                    ),
                    const SizedBox(height: 16),
                    Text(
                      _warningMessage,
                      style: TextStyle(
                        fontSize: isCritical ? 28 : 24,
                        fontWeight: FontWeight.bold,
                        color: isCritical ? Colors.red : Colors.orange,
                      ),
                      textAlign: TextAlign.center,
                    ),
                    if (isCritical) ...[
                      const SizedBox(height: 24),
                      Row(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: [
                          ElevatedButton.icon(
                            onPressed: () {
                              setState(() {
                                _showBrakeWarning = false;
                              });
                            },
                            icon: Icon(Icons.check_circle, size: 24),
                            label: const Text('ACKNOWLEDGED', style: TextStyle(fontSize: 18)),
                            style: ElevatedButton.styleFrom(
                              backgroundColor: Colors.red,
                              foregroundColor: Colors.white,
                              padding: const EdgeInsets.symmetric(horizontal: 32, vertical: 16),
                            ),
                          ),
                        ],
                      ),
                    ],
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}