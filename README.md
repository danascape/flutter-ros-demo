# Flutter ROS Demo

This project is a Flutter application that demonstrates integrating ROS2 with AGL Frontend. It is a part of my GSoC 2025 project https://github.com/danascape/GSoC25-AGL-ROS2

## Getting Started

### Prerequisites

*   Flutter SDK
*   ROS2 Humble
*   Python3

### Installation

1.  Clone the repository:
    ```bash
    git clone https://github.com/danascape/flutter-ros-demo
    cd flutter-ros-demo
    ```

2.  Install Python dependencies:
    ```bash
    pip3 install -r requirements.txt
    ```

3.  Install Flutter dependencies:
    ```bash
    flutter pub get
    ```

### Running the application

These instructions are for running the application on a native x86_64 environment using the [meta-flutter/workspace-automation](https://github.com/meta-flutter/workspace-automation) setup.

1.  Set up the workspace environment. In the `workspace-automation` directory, run:
    ```bash
    source setup_env.sh
    ```

2.  Navigate to this application's directory.

3.  Source your ROS 2 humble environment. If you are using `bash`, run:
    ```bash
    source /opt/ros/humble/setup.bash
    ```
    If you are using `zsh`, run:
    ```bash
    source /opt/ros/humble/setup.zsh
    ```

4.  In a separate terminal, launch the ROS detection system:
    ```bash
    python3 scripts/launch_detection_system.py
    ```

5.  Run the Flutter application:
    ```bash
    flutter run -d linux
    ```
