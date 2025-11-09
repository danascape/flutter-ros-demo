import 'package:flutter/material.dart';
import 'main_navigation.dart';

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter ROS2 App',
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.blue),
        textTheme: const TextTheme().copyWith(
          titleLarge: const TextStyle(fontSize: 28),
          titleMedium: const TextStyle(fontSize: 22),
          titleSmall: const TextStyle(fontSize: 18),
          bodyLarge: const TextStyle(fontSize: 18),
          bodyMedium: const TextStyle(fontSize: 16),
          bodySmall: const TextStyle(fontSize: 14),
          labelLarge: const TextStyle(fontSize: 16),
          labelMedium: const TextStyle(fontSize: 14),
          labelSmall: const TextStyle(fontSize: 12),
        ),
      ),
      home: const MainNavigation(),
    );
  }
}