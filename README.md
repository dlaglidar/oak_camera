# OAK Camera ROS Package

ROS Noetic package for OAK-D Pro W camera, designed for integration with FAST-LIVO2.

## Dependencies

### ROS Packages
```bash
sudo apt update
sudo apt install ros-noetic-image-transport
```

### Python Packages
```bash
pip3 install depthai numpy
```

## Setup

1. Source the workspace:
```bash
source /home/hmd/catkin_ws/devel/setup.bash
```

2. Launch the camera:
```bash
roslaunch oak_camera oak_camera.launch
```

## Published Topics

### Raw Topics (high bandwidth):
- `/oak/rgb/image_raw` - RGB camera (640x360 @ 30 Hz, rgb8)
- `/oak/left/image_raw` - Left stereo camera (640x400 @ 30 Hz, mono8)
- `/oak/right/image_raw` - Right stereo camera (640x400 @ 30 Hz, mono8)
- `/oak/stereo/depth` - Depth image (16UC1, aligned to RGB)
- `/oak/imu` - IMU data (200 Hz, accel + gyro)

### Compressed Topics (recommended for recording):
- `/oak/rgb/image/compressed` - RGB JPEG compressed
- `/oak/left/image/compressed` - Left stereo JPEG compressed
- `/oak/right/image/compressed` - Right stereo JPEG compressed
- `/oak/stereo/depth/compressedDepth` - Depth compressed
- `/oak/imu` - IMU (no compression needed)

## Recording Data

### Record all sensors (compressed):
```bash
rosbag record \
  /oak/rgb/image/compressed \
  /oak/left/image/compressed \
  /oak/right/image/compressed \
  /oak/stereo/depth/compressedDepth \
  /oak/imu \
  /livox/lidar \
  /livox/imu \
  -O full_dataset.bag
```

### Record for FAST-LIVO2 (RGB + LiDAR + IMU):
```bash
rosbag record \
  /oak/rgb/image/compressed \
  /livox/lidar \
  /livox/imu \
  -O fast_livo_dataset.bag
```

### Record stereo + depth for 3D reconstruction:
```bash
rosbag record \
  /oak/left/image/compressed \
  /oak/right/image/compressed \
  /oak/stereo/depth/compressedDepth \
  /oak/imu \
  -O stereo_dataset.bag
```

## Integration with FAST-LIVO2

Remap the camera topic in your FAST-LIVO2 launch file:
```xml
<remap from="/camera/image" to="/oak/rgb/image/compressed"/>
```

## Camera Specifications

**RGB Camera (IMX378):**
- Resolution: 640x360 (downscaled from 1080p)
- Frame rate: 30 FPS
- Encoding: RGB8

**Stereo Cameras:**
- Resolution: 640x400
- Frame rate: 30 FPS
- Encoding: MONO8
- Baseline: ~7.5 cm

**Depth:**
- Range: 0.2m - 10m
- Encoding: 16-bit unsigned (mm)
- Aligned to RGB frame

**IMU:**
- Accelerometer + Gyroscope
- Rate: 200 Hz
