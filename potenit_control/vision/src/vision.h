#ifndef SOCKET_ROS_SETUP_H
#define SOCKET_ROS_SETUP_H

#include <sys/socket.h>
#include <arpa/inet.h>
#include <mutex>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <errno.h>

/// ROS Related///
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
///End of ROS related///

#define PORT 20400

class SocketRos
{
public:
  SocketRos(int argc, char** argv);

  ///Socket Variables////
  char recv_buffer[1024] = {0};
  char send_buffer[1024] = {0};
  bool jobReady = false;

  int sock = 0;
  int valread;
  int socketConnection = 0;
  struct sockaddr_in servAddr;
  int error = 0;
  socklen_t len = sizeof(error);
  ///End of Socket Variables///

  ///ROS Variables///
  bool commAlive = false;
  geometry_msgs::PoseStamped jobPosMsg;
  ///End of ROS Variables

  ///Common Variables///
  bool jobComplete = false;
  bool jobPosReady = false;
  double testX, testY, testZ;
  ///End of Common Variables///

  ///Socket Functions///
  bool spliceString(char* buffer);
  void handleSig(int num);
  int openPort();
  void closePort();
  int transferMsg();
  void jobStatusCheck();
  void setJobPos(float x, float y, float z, float qX, float qY, float qZ, float qW);
  ///End of Socket Funcions

  ///ROS Functions///
  void jobStatusCb(const std_msgs::Int16ConstPtr &msg);
  void initROS();
  ///End of ROS Functions


private:
  int rosArgc;
  char** rosArgv;
  std::mutex jobMutex;
  ros::Subscriber jobStatusSub;
  ros::Publisher jobPosPub;
  ros::Publisher motor1Pub;
  ros::Publisher motor2Pub;

};
//// End of SocketComm Class ///

#endif
