import '../core/object_detection_service.dart';

class DetectionUtils {
  static String formatDetectionResult(DetectionResult result) {
    final timestamp = result.timestamp.toString().substring(11, 19);
    final confidence = (result.confidence * 100).toStringAsFixed(1);
    final bbox = result.boundingBox;

    String formatted = "[$timestamp] ${result.className} (${confidence}%)";

    if (result.distance != null) {
      formatted += " at ${result.distance}";
    }

    if (result.dangerLevel != null) {
      String indicator = _getDangerIndicator(result.dangerLevel!);
      formatted += " $indicator${result.dangerLevel!.toUpperCase()}";
    }

    return formatted;
  }

  static String _getDangerIndicator(String dangerLevel) {
    switch (dangerLevel) {
      case 'critical':
        return '🚨 ';
      case 'warning':
        return '⚠️ ';
      case 'caution':
        return '⚡ ';
      default:
        return '';
    }
  }

  static String formatConfidence(double confidence) {
    return "${(confidence * 100).toStringAsFixed(1)}%";
  }

  static bool isHighConfidence(double confidence) {
    return confidence > 0.7;
  }

  static String getConfidenceColor(double confidence) {
    if (confidence > 0.8) return "green";
    if (confidence > 0.5) return "orange";
    return "red";
  }

  static Map<String, int> countDetectionsByClass(List<DetectionResult> detections) {
    final Map<String, int> counts = {};
    for (final detection in detections) {
      counts[detection.className] = (counts[detection.className] ?? 0) + 1;
    }
    return counts;
  }
}