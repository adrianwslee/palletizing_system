#ifndef SOCKET_DLC_SETUP_H
#define SOCKET_DLC_SETUP_H

#include <arpa/inet.h>
#include <mutex>
#include <real_sense/ros_setup.h>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

volatile sig_atomic_t keyboard_shutdown;

class ros_setup;

class deep_learning
{
public:
  ros_setup* getROS;
  bool socket_connection;
  char arm_status_buffer[1024];
  char dlc2nuc_buffer[1024] = {0};
  std::string send2dlc;

  struct filtered_buffer
  {
    bool ready = false;
    int node = 0;
    bool load = false;
    bool unload = false;
    bool check;
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
  } recvd_buffer;

  std::string convert2char(bool at_sensing, bool ready);
  std::vector<std::string> splice_string(char* buffer);
  int open_port();
  void reset_val();
};

#endif
