#!/usr/bin/env python3
import os
import sys
import copy
import re
import importlib
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
from  sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile

if cv2.__version__ < "4.0.0":
    raise ImportError("Requires opencv >= 4.0, "
                      "but found {:s}".format(cv2.__version__))

class NXPCVCamNode(Node):

    def __init__(self):

        super().__init__("nxp_cv_cam_node")

        camera_image_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Camera image topic.')
        

        resolution_array_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            description='Resolution in pixels [width, height]')

        framerate_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Frames per second as an integer.')

        device_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Device name ex: /dev/video3')
        

        self.declare_parameter("camera_topic", "/NPU/image_raw", 
            camera_image_topic_descriptor)

        self.declare_parameter("resolution", [640, 480], 
            resolution_array_descriptor)

        self.declare_parameter("framerate", 30, 
            framerate_descriptor)

        self.declare_parameter("device", "/dev/video3", 
            device_descriptor)

        self.cameraImageTopic = self.get_parameter("camera_topic").value
        self.resolution = self.get_parameter("resolution").value
        self.framerate = int(self.get_parameter("framerate").value)
        self.device = self.get_parameter("device").value


        #setup CvBridge
        self.bridge = CvBridge()
        
        self.InitTime = int(round(self.get_clock().now().nanoseconds/1000.0))
        
        self.CounterImageMsg = 0

        self.ImagePub = self.create_publisher(Image,
            '{:s}'.format(self.cameraImageTopic), 0)

        videoCaptureString = 'v4l2src device={:s} ! video/x-raw,framerate={:d}/1,width={:d},height={:d} ! appsink'.format(self.device, int(self.framerate), int(self.resolution[0]), int(self.resolution[1]))
        print('VideoCapture: {:s}'.format(videoCaptureString))
        
        self.videoCapture = cv2.VideoCapture(videoCaptureString, cv2.CAP_GSTREAMER)

        self.runCamera()

    def runCamera(self):
        while(self.videoCapture.isOpened()):
            ret, frame = self.videoCapture.read()
            if ret == True:
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "nxp_cv_cam"
                self.ImagePub.publish(msg)
                self.CounterImageMsg += 1
            else:
                print("Video Stream Disconnected.")
                break

        if not self.videoCapture.isOpened():
            Print("Video Capture Device is not open, check settings and connections.")

        return


def main(args=None):
    rclpy.init(args=args)
    node = NXPCVCamNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
