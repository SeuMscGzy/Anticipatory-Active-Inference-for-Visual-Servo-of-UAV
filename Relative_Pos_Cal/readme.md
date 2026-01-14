# Vision & External Perception Module

This directory contains the visual processing stack for the UAV visual servoing system. It implements a robust fiducial marker tracking system using **AprilTags** to estimate the 6-DOF pose of the target, specifically tailored for the Intel RealSense D405 sensor.

## 📂 File Overview

* **`Relative_Pos_Cal_apriltag.cpp/h`**: The core perception node. It detects standard AprilTags (specifically the **tag36h11** family) and computes the relative pose (position and yaw) between the camera and the tag.
* **`set_d405_preset.cpp`**: A hardware configuration utility. It loads specific ISP parameters (exposure, gain, etc.) from a JSON file into the RealSense D405 to optimize image quality for tag detection.
* **`1.json`**: The serialized configuration file for the RealSense D405 "Advanced Mode" settings.

## 🚀 Key Features

1.  **AprilTag Detection**: Integrates the standard `apriltag` C library to detect **tag36h11** markers with high precision.
2.  **6-DOF Pose Estimation**: Solves the PnP (Perspective-n-Point) problem using the tag's known physical size and camera intrinsics to retrieve the full 3D translation ($x, y, z$) and rotation (yaw).
3.  **Hardware Configuration**: Automatically configures the RealSense camera with preset JSON parameters to ensure consistent exposure and contrast.

## 🛠️ Algorithm Details

### 1. Pose Estimation Pipeline (`Relative_Pos_Cal_apriltag`)

The processing pipeline within the `imageCallback` function is as follows:

1.  **Image Conversion**:
    * Converts the raw ROS image message to an OpenCV matrix (`cv_ptr->image`).
    * Converts the image to **grayscale** (`cv::COLOR_BGR2GRAY`), which is required by the AprilTag detector.

2.  **Tag Detection**:
    * Uses `tag36h11_create()` to initialize the detector family.
    * Calls `apriltag_detector_detect` to find all tag candidates in the image.

3.  **Pose Solving**:
    * For each detected tag, `estimate_tag_pose` is called.
    * **Intrinsics**: Uses the camera's focal length ($f_x, f_y$) and principal center ($c_x, c_y$) obtained from the `camera_info` topic.
    * **Tag Size**: The physical tag size is hardcoded as **0.156 meters** (edge length).
    * **Result**: Extracts the translation vector `pose.t` ($x, y, z$) and rotation matrix `pose.R`.

### 2. Camera Setup (`set_d405_preset`)

This node ensures the D405 sensor is in the correct state before flight:
* Reads the `1.json` file string.
* Uses `rs400::advanced_mode` interface to load the JSON settings directly into the device firmware.

## 📡 ROS Interface

### Published Topics
* `/point_with_fixed_delay` (`std_msgs/Float64MultiArray`):
    * **Data**: `[x, y, z, 0, loss_flag, yaw]`
    * **Units**: Meters (position), Radians (yaw).
    * **Description**: The relative position of the tag in the camera frame. `loss_flag` is `1.0` if no tag is detected, `0.0` otherwise.

## ⚙️ Usage

### 1. Hardware Setup
Ensure the D405 camera is connected and the `1.json` file is present in the working directory.

### 2. Launch
First, configure the camera:
```bash
rosrun your_package_name set_d405_preset
```

Then, start the detection node:
```bash
rosrun your_package_name Relative_Pos_Cal
```
