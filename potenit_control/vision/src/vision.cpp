#include "vision.h"

void sigHandler(int sig)
{
  printf("Terminating %d\n", sig);
  exit(1);
}

bool SocketRos::spliceString(char* buffer)
{
  std::string delim = ",";
  std::vector<std::string> strList;
  std::string phrase = std::string(buffer);
  size_t pos = 0;
  std::string token;
  while ((pos = phrase.find(delim)) != std::string::npos)
  {
    token = phrase.substr(0, pos);
    strList.push_back(token);
    phrase.erase(0, pos + delim.length());
  }
  strList.push_back(phrase);
  if(strList.size() == 8)
  {
    setJobPos(stof(strList[1]), stof(strList[2]), stof(strList[3]), stof(strList[4]), stof(strList[5]), stof(strList[6]), stof(strList[7]));
    return true;
  }
  else
  {
    ROS_ERROR("Wrong Coordinate Length");
    return false;
  }
}

int SocketRos::openPort()
{
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    ROS_ERROR("\n Socket creation error \n");
    return -1;
  }

  servAddr.sin_family = AF_INET;
  servAddr.sin_port = htons(PORT);

  // Convert IPv4 and IPv6 addresses from text to binary form
  if (inet_pton(AF_INET, "192.168.137.51", &servAddr.sin_addr) <= 0)
  {
    ROS_ERROR("\nInvalid address/ Address not supported \n");
    return -1;
  }

  // Checks Server Connection
  int countdownServer = 0;
  while (countdownServer < 20)
  {
    if (connect(sock, (struct sockaddr*)&servAddr, sizeof(servAddr)) < 0)
    {
      ROS_WARN("Reconnecting...Attemp #: %d", countdownServer);
    }
    else
    {
      ROS_INFO("A connection established. Proceeding to data transfer");
      break;
    }
    countdownServer++;
    sleep(1);
    std::cout << countdownServer << std::endl;
  }

  if (countdownServer == 19)
  {
    ROS_ERROR("Server Not Availble. Shutting down all programs");
    return -1;
  }
  return 1;
} /// End of openPort ///

void SocketRos::closePort()
{
  close(sock);
  socketConnection = -1;
  shutdown(sock, SHUT_WR);
  printf("Bye Bye");
}

int SocketRos::transferMsg()
{
  socketConnection = openPort();
  if (socketConnection != 1)
  {
    ROS_ERROR("Connection not estabilshed. Please check Server & Clients");
  }

  while (socketConnection)
  {
    ////Temperate Fix for Emergency Break
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = sigHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    ///Do not Edit ABOVE area///

    jobStatusCheck();
    memset(recv_buffer, 0, 1024);
    memset(send_buffer, 0, 1024);

    if (jobReady == true) ///if the job has completed, get ready to receive a next task
    {
      // Sends ready to server
      ROS_INFO("Job Ready");
      std::string completedString = "1,2";
      strcpy(send_buffer, completedString.c_str());
      send(sock, send_buffer, 1024, 0);
      memset(send_buffer, 0, 1024);

      // When ready, receives coordinates from a server
      valread = recv(sock, recv_buffer, 1024, 0);
      std::vector<float> boxCoord;
      if (valread > 0)
      {
        std::cout << recv_buffer << std::endl;
        spliceString(recv_buffer);
        memset(recv_buffer, 0, 1024);
        std::lock_guard<std::mutex> guard(jobMutex);
        jobReady = false;
      }
      else
      {
        ROS_WARN("A received buffer length is not valid");
      }
    }
    /// Add checking socket Communication
  }
  std::cout << "Thread Ends" << std::endl;
  return 0;
} /// End of transferMsg ///

void SocketRos::jobStatusCheck()
{
  std::lock_guard<std::mutex> guard(jobMutex);
  jobReady = jobComplete;
}

void SocketRos::setJobPos(float x, float y, float z, float qX, float qY, float qZ, float qW)
{
  std::lock_guard<std::mutex> guard(jobMutex);
  jobPosMsg.header.frame_id = "camera_link";
  jobPosMsg.header.stamp = ros::Time::now();
  jobPosMsg.pose.position.x = x;
  jobPosMsg.pose.position.y = y;
  jobPosMsg.pose.position.z = z;
  jobPosMsg.pose.orientation.x = qX;
  jobPosMsg.pose.orientation.y = qY;
  jobPosMsg.pose.orientation.z = qZ;
  jobPosMsg.pose.orientation.w = qW;
  jobPosReady = true;
}

SocketRos::SocketRos(int argc, char** argv)
{
  rosArgc = argc;
  rosArgv = argv;
}

void SocketRos::jobStatusCb(const std_msgs::Int16ConstPtr &msg)
{
  ROS_INFO("Beep: %d", msg->data);
  if(msg->data == 1)
  {
    std::lock_guard<std::mutex> guard(jobMutex);
    ROS_INFO("Moving the base motors");
    jobComplete = false;
  }
  else if(msg->data == 2)
  {
    std::lock_guard<std::mutex> guard(jobMutex);
    ROS_INFO("Moving the arm");
    jobComplete = false;
  }
  else if(msg->data == 3)
  {
    std::lock_guard<std::mutex> guard(jobMutex);
    ROS_INFO("Job Complete");
    jobComplete = true;
  }
  else
  {
    ROS_WARN("Unknown command");
  }
}

void SocketRos::initROS()
{
  ros::init(rosArgc, rosArgv, "SocketRos");
  ros::NodeHandle nh;
  jobStatusSub = nh.subscribe<std_msgs::Int16>("/arm/job/status", 10, &SocketRos::jobStatusCb, this);
  motor1Pub = nh.advertise<std_msgs::Int16>("/motor1/cmd", 10);
  motor2Pub = nh.advertise<std_msgs::Int16>("/motor2/cmd", 10);
  jobPosPub = nh.advertise<geometry_msgs::PoseStamped>("/arm/jobPose", 10);

  ros::Rate rate(10);
  while (ros::ok())
  {
    std_msgs::Int16 test1, test2;
    test1.data = 1;
    test2.data = 2;
    if(jobPosReady == true)
    {
      std::lock_guard<std::mutex> guard(jobMutex);
      jobPosPub.publish(jobPosMsg);
      motor1Pub.publish(test1);
      motor2Pub.publish(test2);
      jobReady = false;
      jobPosReady = false;
    }
    commAlive = true;
    ros::spinOnce();
    rate.sleep();
  }
  commAlive = false;
  std::cout << "Thread 2 Ends" << std::endl;
}

int main(int argc, char** argv)
{
  SocketRos* skyCrew = new SocketRos(argc, argv);
  std::thread visionThread(&SocketRos::transferMsg, skyCrew);
  std::thread rosThread(&SocketRos::initROS, skyCrew);
  visionThread.join();
  rosThread.join();

}
