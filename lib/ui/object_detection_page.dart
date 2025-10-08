import 'package:flutter/material.dart';
import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';
import '../core/camera_service.dart';
import '../core/object_detection_service.dart';
import '../utils/detection_utils.dart';
import '../utils/simple_messages.dart';

class ObjectDetectionPage extends StatefulWidget {
  const ObjectDetectionPage({super.key});

  @override
  State<ObjectDetectionPage> createState() => _ObjectDetectionPageState();
}

class _ObjectDetectionPageState extends State<ObjectDetectionPage> {
  final CameraService _cameraService = CameraService();
  final ObjectDetectionService _detectionService = ObjectDetectionService();

  List<String> _detectionResults = [];
  List<String> _systemMessages = [];
  SimpleImageData? _currentImageData;
  MemoryImage? _currentMemoryImage;
  MemoryImage? _previousMemoryImage;
  bool _isConnected = false;
  bool _showBrakeWarning = false;
  String _warningMessage = '';
  double _cameraStreamHeight = 280; // Default height
  int _frameCounter = 0;
  DateTime _lastFrameTime = DateTime.now();

  late StreamSubscription<SimpleImageData> _imageSubscription;
  late StreamSubscription<DetectionResult> _detectionSubscription;

  @override
  void initState() {
    super.initState();
    _initializeServices();
  }

  void _initializeServices() async {
    try {
      await _cameraService.initialize();
      await _detectionService.initialize();

      _setupImageListener();
      _setupDetectionListener();

      if (mounted) {
        setState(() {
          _isConnected = true;
          _systemMessages.add("Camera and detection services initialized");
        });
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          _systemMessages.add("Error initializing services: $e");
        });
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
          _systemMessages.add("Detection received: ${result.className}");

          // Check for brake warning conditions
          _checkBrakeWarning(result);

          if (_detectionResults.length > 50) {
            _detectionResults.removeAt(0);
          }
          if (_systemMessages.length > 20) {
            _systemMessages.removeAt(0);
          }
        });
      }
    });
  }

  void _checkBrakeWarning(DetectionResult result) {
    if (result.isCritical) {
      _showBrakeWarning = true;
      _warningMessage = '🚨 BRAKE! ${result.className.toUpperCase()} AT ${result.distance ?? 'CLOSE RANGE'}';

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
      _showBrakeWarning = true;
      _warningMessage = '⚠️ CAUTION: ${result.className} at ${result.distance ?? 'near'}';

      Future.delayed(Duration(milliseconds: 1500), () {
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
    _cameraService.dispose();
    _detectionService.dispose();
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

  Widget _buildStreamControlCard(BuildContext context) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Stream Controls',
              style: Theme.of(context).textTheme.titleMedium,
            ),
            const SizedBox(height: 8),
            Row(
              children: [
                Text('Stream Size: ', style: Theme.of(context).textTheme.bodySmall),
                Expanded(
                  child: Slider(
                    value: _cameraStreamHeight,
                    min: 200,
                    max: 350, // Limited to 350px for 320x240 content
                    divisions: 6,
                    label: '${_cameraStreamHeight.round()}px',
                    onChanged: (value) {
                      setState(() {
                        _cameraStreamHeight = value;
                      });
                    },
                  ),
                ),
                Text('${_cameraStreamHeight.round()}px',
                     style: Theme.of(context).textTheme.bodySmall),
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
                      color: _isConnected ? Colors.green.withOpacity(0.1) : Colors.grey.withOpacity(0.1),
                      borderRadius: BorderRadius.circular(4),
                      border: Border.all(
                        color: _isConnected ? Colors.green : Colors.grey,
                      ),
                    ),
                    child: Text(
                      '${_currentImageData!.width}x${_currentImageData!.height}',
                      style: TextStyle(
                        color: _isConnected ? Colors.green : Colors.grey,
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
                ],
              ),
            ]
          ],
        ),
      ),
    );
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

    return Positioned.fill(
      child: Container(
        color: isCritical
            ? Colors.red.withOpacity(0.9)
            : Colors.orange.withOpacity(0.8),
        child: Center(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Container(
                padding: const EdgeInsets.all(24),
                decoration: BoxDecoration(
                  color: Colors.white,
                  borderRadius: BorderRadius.circular(16),
                  border: Border.all(
                    color: isCritical ? Colors.red : Colors.orange,
                    width: 4,
                  ),
                ),
                child: Column(
                  children: [
                    Icon(
                      isCritical ? Icons.warning : Icons.info,
                      size: 64,
                      color: isCritical ? Colors.red : Colors.orange,
                    ),
                    const SizedBox(height: 16),
                    Text(
                      _warningMessage,
                      style: TextStyle(
                        fontSize: 32,
                        fontWeight: FontWeight.bold,
                        color: isCritical ? Colors.red : Colors.orange,
                      ),
                      textAlign: TextAlign.center,
                    ),
                    if (isCritical) ...[
                      const SizedBox(height: 16),
                      ElevatedButton(
                        onPressed: () {
                          setState(() {
                            _showBrakeWarning = false;
                          });
                        },
                        style: ElevatedButton.styleFrom(
                          backgroundColor: Colors.red,
                          foregroundColor: Colors.white,
                        ),
                        child: const Text('ACKNOWLEDGED', style: TextStyle(fontSize: 18)),
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