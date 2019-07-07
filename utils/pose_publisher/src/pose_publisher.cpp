/*
 * Copyright (c) 2014, Zhi Yan <zhi.yan@mines-douai.fr>
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of California, Berkeley nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS AND CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  
  double publish_frequency;
  std::string map_frame, base_frame;
  ros::Publisher pose_publisher;
  
  private_nh.param<double>("publish_frequency", publish_frequency, 10);
  private_nh.param<std::string>("map_frame", map_frame, "map");
  private_nh.param<std::string>("base_frame", base_frame, "base_link");
  
  pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
  
  tf::TransformListener listener;
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  
  ros::Rate rate(publish_frequency);
  while(nh.ok()) {
    tf::StampedTransform transform;
    bool tf_ok = true;
    try {
      //listener.waitForTransform(map_frame, base_frame, ros::Time(0), ros::Duration(4.0));
      listener.lookupTransform(map_frame, base_frame,  ros::Time(0), transform);
    } catch(tf::TransformException ex) {
      ROS_ERROR("-------> %s", ex.what());
      tf_ok = false;
    }
    
    if(tf_ok) {
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.stamp = ros::Time::now();
      pose_stamped.header.frame_id = tf_prefix+"/"+map_frame;
      
      pose_stamped.pose.position.x = transform.getOrigin().getX();
      pose_stamped.pose.position.y = transform.getOrigin().getY();
      pose_stamped.pose.position.z = transform.getOrigin().getZ();
      
      pose_stamped.pose.orientation.x = transform.getRotation().getX();
      pose_stamped.pose.orientation.y = transform.getRotation().getY();
      pose_stamped.pose.orientation.z = transform.getRotation().getZ();
      pose_stamped.pose.orientation.w = transform.getRotation().getW();
      
      pose_publisher.publish(pose_stamped);
    }
    
    rate.sleep();
  }
  
  return 0;
}
