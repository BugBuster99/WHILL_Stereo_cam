#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from dynamic_reconfigure.client import Client
from std_msgs.msg import Float32



class ev_updater:
    def __init__(self, slave_node_name='right_slave'):
        self.client = Client(slave_node_name, timeout=30, config_callback=self.config_callback)
        self.ev_sub = rospy.Subscriber("exposure_absolute", Float32, callback=self.ev_callback)

    def ev_callback(self, msg):
        new_exposure_absolute = msg.data
        self.params = self.client.get_configuration()
        self.params['exposure_absolute'] = new_exposure_absolute
        self.client.update_configuration(self.params)

    def config_callback(self, config):
        rospy.loginfo("Slave's exposure_absolute set to {exposure_absolute}".format(**config))

if __name__ == "__main__":
    rospy.init_node("auto_exposure_controller")
    slave_node_name  = rospy.get_param('~slave_node_name', 'right_slave')

    calc = ev_updater(slave_node_name)
    while not rospy.is_shutdown():
        rospy.spin()