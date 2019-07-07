#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from saturnbot_calibration.cfg import demoConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {int_param}, {double_param}, \
                  {str_param}, {bool_param}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("demo_dynamic", anonymous = True)
    srv = Server(demoConfig, callback)
    rospy.spin()
