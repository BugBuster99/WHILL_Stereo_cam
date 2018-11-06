#!/usr/bin/env python
# -*- coding: utf-8 -*-
from math import log, pow, fabs, ceil
import numpy as np
import rospy
from dynamic_reconfigure.client import Client
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2


class ev_calculator:
    def __init__(self, 
                 master_node_name='left_master', 
                 brightness_tgt=128.0, 
                 max_step_size=0.001, 
                 min_step_size=0.0005, 
                 update_interval=5, 
                 max_exposure=0.1):
        self.bridge = CvBridge()
        self.__brightness_tgt  = brightness_tgt
        self.__brightness_tgt_orig = brightness_tgt
        self.__max_step_size   = max_step_size
        self.__min_step_size   = min_step_size
        self.__update_interval = update_interval
        self.__max_exposure    = max_exposure
        self.client = Client(master_node_name, timeout=30, config_callback=None)
        self.image_sub = rospy.Subscriber("src_image", Image, callback=self.img_callback)
        self.image_pub = rospy.Publisher("ev_image", Image, queue_size=1)
        self.ev_pub = rospy.Publisher("exposure_absolute", Float64, queue_size=1)

    def img_callback(self, msg):
        if msg.header.seq % self.__update_interval == 0:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            except CvBridgeError as e:
                print(e)

            height, width = cv_image.shape[:2]
            mask = np.zeros((height, width, 1), np.uint8)
            cv2.circle(mask,(width/2,height/2), height/2, (255, 255, 255), -1)
            masked_image = cv2.bitwise_and(cv_image,mask)
            concat_image = cv2.hconcat([cv_image, masked_image])
            img_msg = self.bridge.cv2_to_imgmsg(concat_image)
            self.image_pub.publish(img_msg)

            #sample = cv_image[240:240+480, 320:320+640]
            max_brightness = np.amax(masked_image)
            #if max_brightness > 250:
            #    self.__brightness_tgt = self.__brightness_tgt - (max_brightness - 250)
            #    if self.__brightness_tgt < self.__brightness_tgt_orig/2:
            #        self.__brightness_tgt = self.__brightness_tgt_orig/2
            #elif max_brightness < 240:
            #    self.__brightness_tgt = self.__brightness_tgt + (240 - max_brightness)
            #    if self.__brightness_tgt > self.__brightness_tgt_orig:
            #        self.__brightness_tgt = self.__brightness_tgt_orig
            brightness_pre = cv2.mean(cv_image, mask=mask)[0]
            #brightness_pre = cv2.mean(sample)[0];
            self.params = self.client.get_configuration()
            old_exposure_absolute = self.params['exposure_absolute']
            new_exposure_absolute = pow(2.0, log(self.__brightness_tgt, 2) - log(brightness_pre, 2) + log(old_exposure_absolute, 2))
            #new_exposure_absolute = old_exposure_absolute * self.__brightness_tgt / brightness_pre
            exposure_step = fabs(new_exposure_absolute - old_exposure_absolute) 
            if exposure_step > self.__min_step_size:
                if exposure_step > self.__max_step_size:
                    if new_exposure_absolute < old_exposure_absolute:
                        new_exposure_absolute = old_exposure_absolute - self.__max_step_size 
                    else:
                        new_exposure_absolute = old_exposure_absolute + self.__max_step_size 
                if new_exposure_absolute > max_exposure:
                    new_exposure_absolute = max_exposure 

                self.ev_pub.publish(new_exposure_absolute)
    

if __name__ == "__main__":
    rospy.init_node("auto_exposure_controller")
    master_node_name = rospy.get_param('~master_node_name', 'left_master')
    brightness_tgt   = rospy.get_param('~brightness_tgt', 128.0)
    max_step_size    = rospy.get_param('~max_step_size', 0.001)
    min_step_size    = rospy.get_param('~min_step_size', 0.0005)
    update_interval  = rospy.get_param('~update_interval', 5)
    max_exposure     = rospy.get_param('~max_exposure', 0.1)

    calc = ev_calculator(master_node_name, brightness_tgt, max_step_size, min_step_size, update_interval, max_exposure)
    while not rospy.is_shutdown():
        rospy.spin()
