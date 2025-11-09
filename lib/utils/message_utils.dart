import 'dart:ffi' as ffi;
import 'package:ffi/ffi.dart';
import 'package:std_msgs/std_msgs.dart';

class MessageUtils {
  static StdMsgsString createFixedStdMsgsString(String message) {
    final msg = StdMsgsString("");

    final utf8String = message.toNativeUtf8();
    msg.data.ref.data = utf8String.cast<ffi.Char>();
    msg.data.ref.size = message.length;
    msg.data.ref.capacity = message.length + 1;

    return msg;
  }

  static bool isValidMessage(String message) {
    return message.trim().isNotEmpty;
  }

  static String formatReceivedMessage(String message) {
    return "Received: $message";
  }

  static String formatSentMessage(String message) {
    return "Sent: $message";
  }
}