# ROS2 Detection System Setup

This directory contains a complete ROS2-based detection system with driver mood detection and front object detection capabilities.

## 🚀 Quick Start

### 1. Install Dependencies
```bash
# Core dependencies
pip install ultralytics opencv-python rclpy

# For emotion detection (optional, will use mock if not available)
pip install fer dlib tensorflow

# For better performance
pip install numpy
```

### 2. Launch the System
```bash
# Option 1: Launch all nodes automatically
python3 scripts/launch_detection_system.py

# Option 2: Launch nodes individually
python3 scripts/detection_mode_controller.py &
python3 scripts/enhanced_road_safety_camera.py &
python3 scripts/driver_mood_detection.py &
```

### 3. Run Flutter App
```bash
flutter run
```

## 📡 ROS2 Topics

| Topic | Purpose | Message Format |
|-------|---------|----------------|
| `/camera_images` | Camera stream | `IMG_DATA:jpeg:width:height:base64_data` |
| `/object_detections` | Front object detection | `DET:class\|confidence\|bbox\|distance\|danger_level` |
| `/driver_detections` | Driver mood/emotion | `DET_DRIVER:type\|emotion\|confidence\|alertness\|fatigue\|safety` |
| `/detection_mode` | Mode switching | JSON: `{"mode": "driver/front", "timestamp": ...}` |
| `/mode_status` | Current mode status | JSON: `{"current_mode": "...", "status": "..."}` |

## 🎯 Detection Modes

### Driver Detection Mode
- **Emotion Recognition**: Happy, sad, angry, neutral, surprise, fear, disgust
- **Alertness Analysis**: High, medium, low alertness levels
- **Fatigue Detection**: Detects driver tiredness and drowsiness
- **Safety Alerts**: Critical warnings for dangerous driving states
- **Phone Detection**: Identifies distracted driving (phone usage)

**Example Output:**
```
[14:23:45] Driver Alert: HIGH (85.2%)
[14:23:46] Emotion: Focused (78% confidence)
[14:23:47] Fatigue Level: LOW
[14:23:48] ⚠️ WARNING: Signs of driver fatigue detected
[14:23:49] 🚨 CRITICAL: Driver appears severely fatigued!
```

### Front Detection Mode
- **Object Detection**: Cars, trucks, buses, motorcycles, bicycles
- **Pedestrian Detection**: People in the roadway
- **Traffic Sign Recognition**: Stop signs, traffic lights
- **Distance Estimation**: Approximate distance to detected objects
- **Collision Risk**: Critical, warning, caution, safe levels

**Example Output:**
```
[14:23:45] Car (89.1%) at 15.2m ⚠️ WARNING
[14:23:46] Person (92.4%) at 8.1m 🚨 CRITICAL
[14:23:47] Traffic Light (76.3%) at 45.0m
[14:23:48] Stop Sign (88.9%) at 12.5m ⚡ CAUTION
```

## 🔄 Mode Switching

The system automatically switches between detection modes based on the Flutter app's radio button selection:

1. **Flutter App** → Radio button selection
2. **Detection Mode Controller** → Receives mode change
3. **All Detection Nodes** → Update their behavior
4. **Results** → Different detection outputs based on mode

## 🏗️ Architecture

```
Flutter App (UI)
     ↓ (mode selection)
Detection Mode Controller
     ↓ (broadcasts mode change)
┌─────────────────┬─────────────────┐
│ Enhanced Camera │ Driver Mood     │
│ (Object Det.)   │ (Emotion Det.)  │
└─────────────────┴─────────────────┘
     ↓ (detection results)
Flutter App (Display Results)
```

## 📁 Files

| File | Purpose |
|------|---------|
| `driver_mood_detection.py` | Driver emotion and fatigue detection |
| `enhanced_road_safety_camera.py` | Camera capture and object detection |
| `detection_mode_controller.py` | Handles mode switching |
| `launch_detection_system.py` | Launcher for all nodes |
| `improved_camera_node.py` | Original camera node (legacy) |
| `road_safety_camera.py` | Original detection node (legacy) |

## 🔧 Configuration

### Camera Settings
- **Resolution**: 640x480 for detection, 480x360 for streaming
- **Frame Rate**: 10 FPS for images, 5 FPS for detection
- **Compression**: JPEG quality 60% for optimal streaming

### Detection Thresholds
- **Object Detection**: 40% confidence minimum
- **Emotion Detection**: 30% confidence minimum
- **Critical Distance**: <10m for vehicles, <8m for pedestrians

### Safety Levels
- **Critical**: Immediate danger, requires urgent action
- **Warning**: Potential hazard, increased attention needed
- **Caution**: Situation awareness required
- **Safe**: Normal driving conditions

## 🐛 Troubleshooting

### Common Issues

**1. Camera not detected**
```bash
# Check available cameras
ls /dev/video*

# Test camera manually
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera Failed')"
```

**2. YOLO model not found**
```bash
# Download YOLO model
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
mv yolov8n.pt /path/to/your/project/
```

**3. Emotion detection not working**
```bash
# Install emotion detection dependencies
pip install fer dlib cmake
```

**4. ROS2 topics not visible**
```bash
# Check ROS2 installation
ros2 topic list

# Source ROS2 if needed
source /opt/ros/humble/setup.bash  # or your ROS2 version
```

### Performance Tips

1. **CPU Usage**: The system uses CPU-only inference for compatibility
2. **Memory**: Each node uses ~100-200MB RAM
3. **Network**: Minimal bandwidth for local ROS2 communication
4. **Storage**: YOLO model requires ~6MB disk space

## 🔍 Monitoring

### Check Node Status
```bash
# List running ROS2 nodes
ros2 node list

# Monitor specific topics
ros2 topic echo /driver_detections
ros2 topic echo /object_detections
```

### View Logs
```bash
# Real-time logs from all nodes
ros2 topic echo /rosout

# Or check individual node output when launched manually
```

## 🎨 Customization

### Adding New Emotions
Edit `driver_mood_detection.py`:
```python
# Add custom emotion analysis in analyze_driver_state()
def analyze_driver_state(self, emotions):
    # Your custom logic here
    pass
```

### Adding New Object Classes
Edit `enhanced_road_safety_camera.py`:
```python
# Add new object classes to road_objects dictionary
self.road_objects = {
    0: 'person',
    1: 'bicycle',
    # Add your new classes here
}
```

### Adjusting Detection Sensitivity
```python
# In detect_*_yolo() functions, modify confidence thresholds
if conf > 0.4:  # Change this value (0.1 to 0.9)
```

## 📚 Further Development

### Integration Ideas
- **GPS Integration**: Correlate detections with location data
- **Weather API**: Adjust detection sensitivity based on conditions
- **Database Logging**: Store detection history for analysis
- **Voice Alerts**: Add audio warnings for critical situations
- **Machine Learning**: Train custom models for specific use cases

### Performance Optimization
- **GPU Support**: Enable CUDA for faster inference
- **Model Optimization**: Use TensorRT or ONNX for better performance
- **Edge Computing**: Deploy on specialized hardware like Jetson Nano
- **Distributed Processing**: Split detection across multiple devices