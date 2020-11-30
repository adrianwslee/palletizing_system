#ifndef SOCKET_ROS_SETUP_H
#define SOCKET_ROS_SETUP_H

#include <real_sense/socket_setup.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <signal.h>
#include <sstream>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <unistd.h>
#include <vector>

class deep_learning;

class ros_setup
{
public:
  deep_learning* getDLC;
  int ros_init_argc;
  char** ros_init_argv;
  std::string arm_status;

  struct return_buffer
  {
    bool sensing_pose = false;
    bool ready = false;
  } send_arm_status;

  void status_cb(const std_msgs::String::ConstPtr& stat_msg);
  bool ros_initialize();

private:
  ros::Publisher camera_pub;
  ros::Publisher cmd_pub;
  ros::Subscriber arm_stat_sub;
  ros::Publisher node_pub;
  std_msgs::Int16 cmd_msg;
  std_msgs::Int16 node_msg;

  static void shutdown_ROS(bool socket_connection)
  {
    if (socket_connection == false)
    {
      ros::shutdown();
    }
  }
};

#endif
