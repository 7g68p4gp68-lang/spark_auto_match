#!/usr/bin/env python3
"""Save one color frame from the robot's ROS camera topic."""

import argparse

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output")
    parser.add_argument("--topic", default="/camera/color/image_raw")
    args = parser.parse_args()
    rospy.init_node("cake_frame_capture", anonymous=True, disable_signals=True)
    message = rospy.wait_for_message(args.topic, Image, timeout=15)
    frame = CvBridge().imgmsg_to_cv2(message, desired_encoding="bgr8")
    if not cv2.imwrite(args.output, frame):
        raise RuntimeError(f"Could not write {args.output}")
    print(f"Saved {frame.shape[1]}x{frame.shape[0]} {args.output}")
    print(f"Timestamp: {message.header.stamp.to_sec():.6f}")


if __name__ == "__main__":
    main()
