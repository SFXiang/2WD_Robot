#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_patrol");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::EmptyRequest>("/move_base/clear_costmaps");

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");

  std::vector<geometry_msgs::Pose> poses;
  geometry_msgs::Pose pose1, pose2, pose3;

  pose1.orientation.w = 0.707;
  pose1.orientation.z = 0.707;
  pose1.position.x = 1.975;
  pose1.position.y = 0.024;
  poses.push_back(pose1);

  pose2.orientation.w = -0.417;
  pose2.orientation.z = 0.907;
  pose2.position.x = -10.79;
  pose2.position.y = 0,5;
  poses.push_back(pose2);

  pose3.orientation.w = 0.967;
  pose3.orientation.z = 0.253;
  pose3.position.x = -1.55;
  pose3.position.y = 4.84;
  poses.push_back(pose3);

  std::vector<geometry_msgs::Pose>::iterator it = poses.begin();

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.pose = *it;
  ac.sendGoal(goal);

  ros::Rate rate(1);
  while (ros::ok() && ac.waitForResult())
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());

    if (state == state.SUCCEEDED)
    {

      it++;

      if (it == poses.end())
      {
        it = poses.begin();

      }

      ros::Rate(0.2).sleep();
      goal.target_pose.pose = *it;
      ac.sendGoal(goal);
    }
    else if (state == state.ABORTED)
    {

      getchar();
      std_srvs::Empty trig;
      client.call(trig);

      goal.target_pose.pose = *it;
      ac.sendGoal(goal);

      ROS_INFO("Resumed.");
    }
    rate.sleep();
  }

  return 0;
}
