#include <real_sense/ros_setup.h>
#include <real_sense/socket_setup.h>

#define PORT 20400

bool connection;

void keyboard_interrupt(int signum) { connection = false; };

std::vector<std::string> deep_learning::splice_string(char* buffer)
{
  std::string delim = ",";
  std::vector<std::string> list;
  std::string phrase = std::string(buffer);
  size_t pos = 0;
  std::string token;
  while ((pos = phrase.find(delim)) != std::string::npos)
  {
    token = phrase.substr(0, pos);
    list.push_back(token);
    phrase.erase(0, pos + delim.length());
  }
  list.push_back(phrase);
  recvd_buffer.ready = stoi(list[0]);  // Ready == True when the robot is ready to move
  recvd_buffer.node = stoi(list[1]);   // Node - gets the destination node number
  recvd_buffer.load = stoi(list[2]);   // Load == True when the robot should load an object
  recvd_buffer.unload = stoi(list[3]); // Unload == True when the robot should unload an object
  recvd_buffer.check = stoi(list[4]);  // Check == True when the camera recvs the sensing msg(?)
  recvd_buffer.x = stof(list[5]);
  recvd_buffer.y = stof(list[6]);
  recvd_buffer.z = stof(list[7]);
  return list;
}

void deep_learning::reset_val()
{
  recvd_buffer.ready = 0;  // Ready == True when the robot is ready to move
  recvd_buffer.node = 0;   // Node - gets the destination node number
  recvd_buffer.load = 0;   // Load == True when the robot should load an object
  recvd_buffer.unload = 0; // Unload == True when the robot should unload an object
  recvd_buffer.check = 0;  // Check == True when the camera recvs the sensing msg(?)
  recvd_buffer.x = 0;
  recvd_buffer.y = 0;
  recvd_buffer.z = 0;
}

std::string deep_learning::convert2char(bool at_sensing, bool ready)
{
  std::string arm2pc;
  arm2pc = std::to_string(at_sensing) + "," + std::to_string(ready);
  return arm2pc;
}

int deep_learning::open_port()
{
  int sock = 0; // socketfd
  int valread;
  struct sockaddr_in serv_addr;
  const char* connection_check = "connect";
  int error = 0;
  socklen_t len = sizeof(error);

  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    ROS_ERROR("\n Socket creation error \n");
    return -1;
  }

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(PORT);

  // Convert IPv4 and IPv6 addresses from text to binary form
  if (inet_pton(AF_INET, "192.168.126.201", &serv_addr.sin_addr) <= 0)
  {
    ROS_ERROR("\nInvalid address/ Address not supported \n");
    return -1;
  }

  // Checks Server Connection
  int countdown_server = 0;
  while (countdown_server < 20)
  {
    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0)
    {
      ROS_WARN("Reconnecting...");
    }
    else
    {
      ROS_INFO("A connection with a server established. Proceeding to data transfer");
      break;
    }
    countdown_server++;
    sleep(1);
    std::cout << countdown_server << std::endl;
  }

  if (countdown_server == 19)
  {
    ROS_ERROR("Server Not Availble. Shutting down all programs");
    connection = false;
    return -1;
  }

  /// Start of the communication
  while (connection)
  {
    signal(SIGINT, keyboard_interrupt); // Ctrl+C stop
    valread = recv(sock, dlc2nuc_buffer, 1024, 0);
    std::vector<std::string> recvd_cmd = splice_string(dlc2nuc_buffer);

#if 1
    for (int i = 0; i < recvd_cmd.size(); i++)
    {
      std::cout << "Recvd Command from DLC: " << recvd_cmd[i] << std::endl;
    }
#endif

    /// Sending Arm Status to DLC
    if ((recvd_buffer.load == 1 && recvd_buffer.unload == 0) || (recvd_buffer.load == 0 && recvd_buffer.unload == 1)) // TO DO: Changed Load && Unload == 1 to Load || Unload and remove the below code
    {

      // Clears all buffers
      memset(arm_status_buffer, 0, 1024);
      memset(dlc2nuc_buffer, 0, 1024);

      //Waits until "sensing" is received by ROS
      while (!getROS->send_arm_status.sensing_pose)
      {
        sleep(1);
        signal(SIGINT, keyboard_interrupt); // Ctrl+C stop
      }
      ROS_DEBUG("At Sening Pose: %d, %d", getROS->send_arm_status.sensing_pose, getROS->send_arm_status.ready);

      //Reformat message to buffer string type and sends to DLC
      send2dlc = convert2char(getROS->send_arm_status.sensing_pose, getROS->send_arm_status.ready);
      strcpy(arm_status_buffer, send2dlc.c_str());
      send(sock, arm_status_buffer, 1024, 0);
      memset(arm_status_buffer, 0, 1024);

      // Recieve Coordinates
      valread = recv(sock, dlc2nuc_buffer, 1024, 0);
      std::vector<std::string> recvd_cmd = splice_string(dlc2nuc_buffer);
      ROS_INFO("Received Coordinates: %s", dlc2nuc_buffer);

      //Waits until Ready
      while (!getROS->send_arm_status.ready)
      {
        sleep(1);
        ROS_DEBUG("%d", getROS->send_arm_status.ready);
        signal(SIGINT, keyboard_interrupt); // Ctrl+C stop
      }
      ROS_DEBUG("Loaded / Unloaded : %d, %d", getROS->send_arm_status.sensing_pose, getROS->send_arm_status.ready);
      send2dlc = convert2char(getROS->send_arm_status.sensing_pose, getROS->send_arm_status.ready);
      strcpy(arm_status_buffer, send2dlc.c_str());
      send(sock, arm_status_buffer, 1024, 0);
      memset(arm_status_buffer, 0, 1024);
      sleep(1);
    }
    else
    {
      ROS_INFO("Waiting for the command");
    }

    ////Checking Socket Communication
    socklen_t len = sizeof(error);
    /// get socket option : return = 0 success reading -1 error check errno
    int retval = getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len);
    if (retval != 0)
    {
      fprintf(stderr, "error getting error code: %s \n", strerror(retval));
      ROS_WARN("Error getting ERROR");
      connection = false;
    }
    if (error != 0)
    {
      fprintf(stderr, "socket error: %s\n", strerror(error));
      ROS_WARN("Socket ERROR");
      connection = false;
    }

    // Clear buffer
    memset(dlc2nuc_buffer, 0, 1024);
    memset(arm_status_buffer, 0, 1024);
    sleep(1);
  } // End of while loop
  close(sock);
  shutdown(sock, SHUT_WR);
}


//////////////////////////////////////////////ROS////////////////////////////////////////////////

void ros_setup::status_cb(const std_msgs::String::ConstPtr& stat_msg)
{
  arm_status.clear();
  arm_status = stat_msg->data;
}

bool ros_setup::ros_initialize()
{
  ros::init(ros_init_argc, ros_init_argv, "etri_ws");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  camera_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_pose", 1);
  cmd_pub = nh.advertise<std_msgs::Int16>("/arm/cmd", 1);
  arm_stat_sub = nh.subscribe<std_msgs::String>("/arm/status", 1, &ros_setup::status_cb, this);
  node_pub = nh.advertise<std_msgs::Int16>("/goal_node", 1);
  cmd_msg.data = 3;
  bool sentCheckCoordinates = false;
  bool sentCheckCmd = false;
  bool sentCheckNodeNumber = false;
  while (ros::ok())
  {
    // Shutdown ROS when Socket Connection is False
    shutdown_ROS(connection);
    geometry_msgs::PoseStamped coordinates;
    coordinates.header.frame_id = "/camera";
    if (arm_status == "sensing")
    {
      send_arm_status.sensing_pose = true;
      send_arm_status.ready = false;
      // ROS_INFO("Actual : %d", send_arm_status.sensing_pose);
      //        recv_sensing = true; //For now
      if (getDLC->recvd_buffer.check == true)
      {
	coordinates.pose.position.x = getDLC->recvd_buffer.x - 0.01; // camera object detection offset  - 0.005
        coordinates.pose.position.y = getDLC->recvd_buffer.y - 0.04;  // camera object detection offset
        coordinates.pose.position.z = getDLC->recvd_buffer.z;
        coordinates.pose.orientation.w = 1;
#if 0
        std::cout << "Received Coordinates " << coordinates.pose.position.x << " " << coordinates.pose.position.y << " " << coordinates.pose.position.z << std::endl;
#endif

        if (!sentCheckCoordinates)
        {
          camera_pub.publish(coordinates);
          sentCheckCoordinates = true;
        }
      }
      else
      {
        ROS_WARN("Error, No coordinates received");
      }
    }
    else
    {
      coordinates.pose.position.x = 0;
      coordinates.pose.position.y = 0;
      coordinates.pose.position.z = 0;
      coordinates.pose.orientation.w = 1;
      sentCheckCoordinates = false;
    } // End of "sensing"
    if (arm_status == "standby")
    {
      send_arm_status.ready = false;
    }
    if (arm_status == "loaded")
    {
      send_arm_status.sensing_pose = false;
      send_arm_status.ready = true;
      getDLC->reset_val();
      sleep(1);
      arm_status.clear();
      ROS_DEBUG("Loaded called");
    }
    if (arm_status == "unloaded") // Stops
    {
      send_arm_status.sensing_pose = false;
      send_arm_status.ready = true;
      sentCheckCmd = false;
      getDLC->reset_val();
      sleep(1);
      cmd_msg.data = 10; //When adding more funcitons, please check this
      arm_status.clear();
      ROS_DEBUG("Unloaded called");
      if (cmd_msg.data == 10) //Please match above commented section cmd_msg.data with this
      {
        sleep(1);
        send_arm_status.sensing_pose = false;
        send_arm_status.ready = false;
      }
    }

    //Set Cmd (Load / Unload / Standby)
    if (getDLC->recvd_buffer.load == 1 && getDLC->recvd_buffer.unload == 0) // Load
    {
      cmd_msg.data = 1;
      if(!sentCheckCmd)
      {
          cmd_pub.publish(cmd_msg);
          sentCheckCmd = true;
      }
    }
    if (getDLC->recvd_buffer.unload == 1 && getDLC->recvd_buffer.load == 0) // Unload
    {
      cmd_msg.data = 2;
      if(!sentCheckCmd)
      {
          cmd_pub.publish(cmd_msg);
          sentCheckCmd = true;
      }
    }
    if (cmd_msg.data == 3) // Standby position which resets all arm_status
    {
      send_arm_status.sensing_pose = false;
      send_arm_status.ready = false;
    }

    //Set Node number
    if (getDLC->recvd_buffer.node != 0)
    {
      node_msg.data = getDLC->recvd_buffer.node;
    }
    else
    {
        ROS_WARN("No Node Number given");
    }

    //Publish Node Number
    node_pub.publish(node_msg);
  
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  connection = true;
  deep_learning dlc;
  ros_setup ros_start;
  ros_start.ros_init_argc = argc;
  ros_start.ros_init_argv = argv;

  // Share Data between DLC & ROS
  ros_start.getDLC = &dlc;
  dlc.getROS = &ros_start;

  std::thread dlc_thread(&deep_learning::open_port, &dlc);
  std::thread ros_thread(&ros_setup::ros_initialize, &ros_start);
  dlc_thread.join();
  ros_thread.join();
  return 0;
}
