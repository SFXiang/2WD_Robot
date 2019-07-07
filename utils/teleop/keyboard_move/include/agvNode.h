/*
 * agvNode.h
 *
 *  Created on: Sep 21, 2016
 *      Author: root
 */

#ifndef AGVNODE_H_
#define AGVNODE_H_

class AgvNode {

public:
	AgvNode();
	~AgvNode();
	void keyboardLoop();
	void stopRobot();

private:
	double walk_vel_;
	double run_vel_;
	double yaw_rate_;
	double yaw_rate_run_;

	geometry_msgs::Twist cmdvel_;
	double vx_tmp;
	double vw_tmp;
	double linear_Step;
	double rotate_Step;
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::NodeHandle n_private;
};




#endif /* AGVNODE_H_ */
