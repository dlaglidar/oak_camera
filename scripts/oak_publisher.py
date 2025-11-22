#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import depthai as dai
import sys

sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')

def main():
    rospy.init_node('oak_camera_publisher', anonymous=True)
    rgb_pub = rospy.Publisher('/oak/rgb/image_raw', Image, queue_size=10)

    rospy.loginfo("Starting OAK camera...")

    # Create pipeline
    with dai.Pipeline() as pipeline:
        # Create camera node
        cam_rgb = pipeline.create(dai.node.Camera).build()

        # Configure RGB camera output
        rgb_queue = cam_rgb.requestOutput(size=(640, 480), fps=30).createOutputQueue()

        # Start pipeline
        pipeline.start()

        rospy.loginfo("OAK camera started! Publishing to /oak/rgb/image_raw")

        while pipeline.isRunning() and not rospy.is_shutdown():
            rgb_frame = rgb_queue.tryGet()
            if rgb_frame is not None:
                frame = rgb_frame.getCvFrame()

                # Create ROS message
                msg = Image()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "oak_rgb_camera"
                msg.height = frame.shape[0]
                msg.width = frame.shape[1]
                msg.encoding = "rgb8"
                msg.is_bigendian = 0
                msg.step = frame.shape[1] * 3
                msg.data = frame.tobytes()

                rgb_pub.publish(msg)

            rospy.sleep(0.001)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
