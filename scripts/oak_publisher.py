#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, Imu
import depthai as dai
import numpy as np
import sys

sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')

def main():
    rospy.init_node('oak_camera_publisher', anonymous=True)

    # Publishers for all camera streams
    rgb_pub = rospy.Publisher('/oak/rgb/image_raw', Image, queue_size=10)
    left_pub = rospy.Publisher('/oak/left/image_raw', Image, queue_size=10)
    right_pub = rospy.Publisher('/oak/right/image_raw', Image, queue_size=10)
    depth_pub = rospy.Publisher('/oak/stereo/depth', Image, queue_size=10)
    imu_pub = rospy.Publisher('/oak/imu', Imu, queue_size=10)

    rospy.loginfo("Starting OAK-D Pro W camera with all sensors...")

    # Create pipeline using newer API
    with dai.Pipeline() as pipeline:
        # Create camera nodes with newer API
        cam_rgb = pipeline.create(dai.node.Camera).build(
            source="color",
            size=(640, 360),
            fps=30
        )

        cam_left = pipeline.create(dai.node.Camera).build(
            source="left",
            size=(640, 400),
            fps=30
        )

        cam_right = pipeline.create(dai.node.Camera).build(
            source="right",
            size=(640, 400),
            fps=30
        )

        # Create stereo depth node
        stereo = pipeline.create(dai.node.StereoDepth)

        # Link cameras to stereo
        cam_left.requestOutput().link(stereo.left)
        cam_right.requestOutput().link(stereo.right)

        # Configure stereo depth
        config = stereo.initialConfig.get()
        config.postProcessing.median.set(dai.MedianFilter.KERNEL_7x7)
        stereo.initialConfig.set(config)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(False)

        # Create IMU node
        imu_node = pipeline.create(dai.node.IMU)
        imu_node.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW,
                                  dai.IMUSensor.GYROSCOPE_RAW],
                                 200)  # 200 Hz
        imu_node.setBatchReportThreshold(1)
        imu_node.setMaxBatchReports(10)

        # Request outputs and create queues
        rgb_queue = cam_rgb.requestOutput(size=(640, 360), fps=30).createOutputQueue()
        left_queue = cam_left.requestOutput(size=(640, 400), fps=30).createOutputQueue()
        right_queue = cam_right.requestOutput(size=(640, 400), fps=30).createOutputQueue()
        depth_queue = stereo.requestOutput(size=(640, 360)).createOutputQueue()

        # Create XLink output for IMU
        xout_imu = pipeline.create(dai.node.XLinkOut)
        xout_imu.setStreamName("imu")
        imu_node.out.link(xout_imu.input)

        # Start pipeline
        pipeline.start()

        # Get IMU queue
        device = pipeline.getDevice()
        imu_queue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)

        rospy.loginfo("OAK-D Pro W started! Publishing:")
        rospy.loginfo("  - RGB: /oak/rgb/image_raw")
        rospy.loginfo("  - Left: /oak/left/image_raw")
        rospy.loginfo("  - Right: /oak/right/image_raw")
        rospy.loginfo("  - Depth: /oak/stereo/depth")
        rospy.loginfo("  - IMU: /oak/imu")

        while pipeline.isRunning() and not rospy.is_shutdown():
            # Process RGB
            rgb_frame = rgb_queue.tryGet()
            if rgb_frame is not None:
                frame = rgb_frame.getCvFrame()
                msg = Image()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "oak_rgb_camera_optical_frame"
                msg.height = frame.shape[0]
                msg.width = frame.shape[1]
                msg.encoding = "rgb8"
                msg.is_bigendian = 0
                msg.step = frame.shape[1] * 3
                msg.data = frame.tobytes()
                rgb_pub.publish(msg)

            # Process Left
            left_frame = left_queue.tryGet()
            if left_frame is not None:
                frame = left_frame.getCvFrame()
                msg = Image()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "oak_left_camera_optical_frame"
                msg.height = frame.shape[0]
                msg.width = frame.shape[1]
                msg.encoding = "mono8"
                msg.is_bigendian = 0
                msg.step = frame.shape[1]
                msg.data = frame.tobytes()
                left_pub.publish(msg)

            # Process Right
            right_frame = right_queue.tryGet()
            if right_frame is not None:
                frame = right_frame.getCvFrame()
                msg = Image()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "oak_right_camera_optical_frame"
                msg.height = frame.shape[0]
                msg.width = frame.shape[1]
                msg.encoding = "mono8"
                msg.is_bigendian = 0
                msg.step = frame.shape[1]
                msg.data = frame.tobytes()
                right_pub.publish(msg)

            # Process Depth
            depth_frame = depth_queue.tryGet()
            if depth_frame is not None:
                frame = depth_frame.getCvFrame()
                msg = Image()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "oak_rgb_camera_optical_frame"
                msg.height = frame.shape[0]
                msg.width = frame.shape[1]
                msg.encoding = "16UC1"
                msg.is_bigendian = 0
                msg.step = frame.shape[1] * 2
                msg.data = frame.tobytes()
                depth_pub.publish(msg)

            # Process IMU
            imu_data = imu_queue.tryGet()
            if imu_data is not None:
                imu_packets = imu_data.packets
                for imu_packet in imu_packets:
                    accel = imu_packet.acceleroMeter
                    gyro = imu_packet.gyroscope

                    msg = Imu()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "oak_imu_frame"

                    # Linear acceleration (m/s^2)
                    msg.linear_acceleration.x = accel.x
                    msg.linear_acceleration.y = accel.y
                    msg.linear_acceleration.z = accel.z

                    # Angular velocity (rad/s)
                    msg.angular_velocity.x = gyro.x
                    msg.angular_velocity.y = gyro.y
                    msg.angular_velocity.z = gyro.z

                    # Set covariances to -1 (unknown)
                    msg.linear_acceleration_covariance = [-1.0] * 9
                    msg.angular_velocity_covariance = [-1.0] * 9
                    msg.orientation_covariance = [-1.0] * 9

                    imu_pub.publish(msg)

            rospy.sleep(0.001)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
