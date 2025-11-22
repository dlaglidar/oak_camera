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

- `/oak/rgb/image_raw` - Raw RGB images (sensor_msgs/Image)
- `/camera/image/compressed` - Compressed JPEG images (sensor_msgs/CompressedImage)

## Integration with FAST-LIVO2

### Option 1: Use compressed images (recommended for smaller bag files)
In your FAST-LIVO2 launch file, remap the camera topic:
```xml
<remap from="/camera/image" to="/camera/image/compressed"/>
```

### Option 2: Use raw images
```xml
<remap from="/camera/image" to="/oak/rgb/image_raw"/>
```

## Recording Data

### Compressed (smaller files):
```bash
rosbag record /camera/image/compressed /livox/lidar /livox/imu -O dataset.bag
```

### Raw images:
```bash
rosbag record /oak/rgb/image_raw /livox/lidar /livox/imu -O dataset.bag
```

## Camera Specifications

- Resolution: 640x480
- Frame rate: 30 FPS
- Encoding: RGB8
- Frame ID: oak_rgb_camera
