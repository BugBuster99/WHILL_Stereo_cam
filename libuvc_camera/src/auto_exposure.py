#!/usr/bin/env python
# -*- codign: utf-8 -*-
from math import log2
import rospy
from dynamic_reconfigure.client import Client
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2


class ev_calculator:
    def __init__(self, camera_node_name='left_master', brightness_tgt=128):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/src_image", Image, callback=self.img_callback)
        self.client = Client(camera_node_name, timeout=30, config_callback=self.config_callback)
        self.__brightness_tgt = brightness_tgt

    def img_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except CvBridgeError as e:
            print(e)


        brightness_pre = cv2.mean(cv_image)
        params = self.client.get_configuration()
        old_exposure_absolute = params['exposure_absolute']
        new_exposure_absolute = old_exposure_absolute + log2(brightness_pre) - log2(self.__brightness_tgt)
        params['exposure_absolute'] = new_exposure_absolute
        self.client.update_configuration(params)

    def config_callback(self, config):
        rospy.loginfo("Config set to {exposure_absolute}".format(**config))
    

if __name__ == "__main__":
    rospy.init_node("auto_exposure_controller")
    camera_node_name = rospy.get_param('camera_node_name', 'left_master')
    brightness_tgt   = rospy.get_param('brightness_tgt', 128)
    calc = ev_calculator(camera_node_name, brightness_tgt)
    while not rospy.is_shutdown():
        rospy.spin()