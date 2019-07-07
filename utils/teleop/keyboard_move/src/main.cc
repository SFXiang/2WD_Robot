/*
 * main.cc
 *
 *  Created on: Sep 21, 2016
 *      Author: root
 */

#include <termios.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "TimerDiff.h"
#include "agvNode.h"

void shutdown(int sig)
{
  //cmdVelPub.publish(geometry_msgs::Twist());
  ROS_INFO("keyboard_move stop ...");
  ros::shutdown();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "keyboard_move");
	ros::NodeHandle node;
	double m_haicheng_;

	ros::Rate loopRate(40);
	signal(SIGINT, shutdown);
	ROS_INFO("keyboard_move start ...");

	AgvNode agv;

	boost::thread t = boost::thread(boost::bind(&AgvNode::keyboardLoop,&agv));
	ros::spin();

	t.interrupt();
	t.join();
	agv.stopRobot();

	tcsetattr(kfd, TCSANOW, &cooked);
	return 0;
	while (ros::ok())
	{
		loopRate.sleep();
	}

	return 0;
}



