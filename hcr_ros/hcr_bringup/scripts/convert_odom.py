#!/usr/bin/env python

""" convert_odom.py
    Convert the message on /robot_pose_ekf/odom_combined topic whose type is
    'PoseWithCovarianceStamped' to standard Odometry message.
    Copyright (C) 2017  Liu Chaoyang

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import roslib
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class OdomConverter():
    def __init__(self, base_frame='base_footprint'):
        # Give the node a name
        rospy.init_node('odom_converter', anonymous=False)
        self.base_frame = base_frame
        # Publisher of type nav_msgs/Odometry
        self.odom_pub = rospy.Publisher('output', Odometry, queue_size=10)

        # Wait for the /odom topic to become available
        rospy.wait_for_message('input', PoseWithCovarianceStamped)

        # Subscribe to the /robot_pose_ekf/odom_combined topic
        rospy.Subscriber('input', PoseWithCovarianceStamped, self.pub_odom)

        rospy.loginfo("Publishing combined odometry on /output")

    def pub_odom(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = self.base_frame
        odom.pose = msg.pose

        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        OdomConverter()
        rospy.spin()
    except:
        pass
