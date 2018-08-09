#!/usr/bin/env python
# -*- coding: utf-8 -*-
from math import log, pow, fabs
import rospy
from dynamic_reconfigure.client import Client
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2


class ev_calculator:
    def __init__(self, camera_node_name='left_master', brightness_tgt=128.0, step_size=0.001, update_interval=5, max_exposure=0.1):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("src_image", Image, callback=self.img_callback)
        self.client = Client(camera_node_name, timeout=30, config_callback=self.config_callback)
        self.__brightness_tgt  = brightness_tgt
        self.__step_size       = step_size
        self.__update_interval = update_interval
        self.__max_exposure    = max_exposure

    def img_callback(self, msg):
        if msg.header.seq % self.__update_interval == 0:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            except CvBridgeError as e:
                print(e)


            sample = cv_image[240:240+480, 320:320+640]
            brightness_pre = cv2.mean(sample)[0]
            params = self.client.get_configuration()
            old_exposure_absolute = params['exposure_absolute']
            new_exposure_absolute = pow(2.0, log(self.__brightness_tgt, 2) - log(brightness_pre, 2) + log(old_exposure_absolute, 2))
            #new_exposure_absolute = old_exposure_absolute * self.__brightness_tgt / brightness_pre
            if fabs(new_exposure_absolute - old_exposure_absolute) > self.__step_size:
                if new_exposure_absolute < old_exposure_absolute:
                    new_exposure_absolute = old_exposure_absolute - self.__step_size 
                else:
                    new_exposure_absolute = old_exposure_absolute + self.__step_size 
            if new_exposure_absolute > max_exposure:
                new_exposure_absolute = max_exposure 
            #rospy.loginfo("current brightness = %f", brightness_pre)
            #rospy.loginfo("tgt BL = %f", log(brightness_tgt, 2))
            #rospy.loginfo("log BL = %f", log(brightness_pre, 2))
            #rospy.loginfo("old EV = %f", old_exposure_absolute)
            #rospy.loginfo("new EV = %f", new_exposure_absolute)

            params['exposure_absolute'] = new_exposure_absolute
            self.client.update_configuration(params)

    def config_callback(self, config):
        rospy.logdebug("exposure_absolute set to {exposure_absolute}".format(**config))
    

if __name__ == "__main__":
    rospy.init_node("auto_exposure_controller")
    camera_node_name = rospy.get_param('~camera_node_name', 'left_master')
    brightness_tgt   = rospy.get_param('~brightness_tgt', 128.0)
    step_size        = rospy.get_param('~step_size', 0.001)
    update_interval  = rospy.get_param('~update_interval', 5)
    max_exposure     = rospy.get_param('~max_exposure', 0.1)

    calc = ev_calculator(camera_node_name, brightness_tgt, step_size, update_interval, max_exposure)
    while not rospy.is_shutdown():
        rospy.spin()
