#!/usr/bin/env python3

import subprocess
import sys
import time
import signal
import os

def launch_node(script_name, node_name):
    """Launch a ROS2 node"""
    try:
        print(f"Launching {node_name}...")
        process = subprocess.Popen([
            'python3', script_name
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        return process
    except Exception as e:
        print(f"Failed to launch {node_name}: {e}")
        return None

def main():
    print("ROS2 Detection System Launcher")
    print("=" * 50)

    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Node configurations
    nodes = [
        {
            'script': os.path.join(script_dir, 'detection_mode_controller.py'),
            'name': 'Detection Mode Controller',
            'description': 'Handles mode switching between driver and front detection'
        },
        {
            'script': os.path.join(script_dir, 'enhanced_road_safety_camera.py'),
            'name': 'Enhanced Road Safety Camera',
            'description': 'Camera capture and front object detection'
        },
        {
            'script': os.path.join(script_dir, 'driver_mood_detection.py'),
            'name': 'Driver Mood Detection',
            'description': 'Driver emotion and fatigue monitoring'
        }
    ]

    processes = []

    try:
        # Launch all nodes
        for node in nodes:
            print(f"\n  {node['name']}")
            print(f"   {node['description']}")

            if os.path.exists(node['script']):
                process = launch_node(node['script'], node['name'])
                if process:
                    processes.append(process)
                    print(f"    Launched successfully (PID: {process.pid})")
                    time.sleep(1)  # Give each node time to start
                else:
                    print(f"     Failed to launch")
            else:
                print(f"     Script not found: {node['script']}")

        if not processes:
            print("\n  No nodes were launched successfully!")
            return

        print(f"\n  Successfully launched {len(processes)} nodes!")
        print("\n  ROS2 Topics:")
        print("   /camera_images        - Camera stream")
        print("   /object_detections    - Front object detection results")
        print("   /driver_detections    - Driver mood/emotion results")
        print("   /detection_mode       - Mode switching commands")
        print("   /mode_status          - Current mode status")

        print("\n  Mode Switching:")
        print("   Use the Flutter app radio buttons to switch between:")
        print("   • Driver Detection  - Emotion and fatigue monitoring")
        print("   • Front Detection   - Road object detection")

        print("\n  Dependencies (install if needed):")
        print("   pip install ultralytics opencv-python")
        print("   pip install fer dlib  # For emotion detection")

        print(f"\n  System running with {len(processes)} nodes...")
        print("Press Ctrl+C to stop all nodes")

        # Wait for all processes and monitor their health
        while True:
            time.sleep(1)

            # Check if any process has died
            for i, process in enumerate(processes):
                if process.poll() is not None:
                    print(f"\n   Node {nodes[i]['name']} has stopped (exit code: {process.returncode})")

                    # Try to restart the node
                    print(f"  Attempting to restart {nodes[i]['name']}...")
                    new_process = launch_node(nodes[i]['script'], nodes[i]['name'])
                    if new_process:
                        processes[i] = new_process
                        print(f"     Restarted successfully")
                    else:
                        print(f"     Failed to restart")

    except KeyboardInterrupt:
        print(f"\n\n  Shutting down {len(processes)} nodes...")

        # Terminate all processes gracefully
        for i, process in enumerate(processes):
            if process.poll() is None:  # Process is still running
                print(f"   Stopping {nodes[i]['name']}...")
                process.terminate()

                # Wait up to 3 seconds for graceful shutdown
                try:
                    process.wait(timeout=3)
                    print(f"     {nodes[i]['name']} stopped gracefully")
                except subprocess.TimeoutExpired:
                    print(f"     Force killing {nodes[i]['name']}...")
                    process.kill()
                    process.wait()

        print("🏁 All nodes stopped. Goodbye!")

    except Exception as e:
        print(f"\n  Launcher error: {e}")
        # Clean up any running processes
        for process in processes:
            if process.poll() is None:
                process.terminate()

if __name__ == '__main__':
    main()
