#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <string>
#define PORT 20400

void getCoordinates() {}

char recv_buffer[1024] = {0};
char send_buffer[1024] = {0};

int main(int argc, char const* argv[])
{
  int server_fd, new_socket, valread;
  struct sockaddr_in address;
  int opt = 1;
  int addrlen = sizeof(address);
  char buffer[1024] = {0};
  char* hello = "1,1,1,0,1,0.05,-0.05,-0.05";

  // Creating socket file descriptor
  if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
  {
    perror("socket failed");
    exit(EXIT_FAILURE);
  }

  // Forcefully attaching socket to the port 8080
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt,
                 sizeof(opt)))
  {
    perror("setsockopt");
    exit(EXIT_FAILURE);
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(PORT);

  // Forcefully attaching socket to the port 8080
  if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0)
  {
    perror("bind failed");
    exit(EXIT_FAILURE);
  }
  else
  {
    printf("connection success");
  }
  if (listen(server_fd, 3) < 0)
  {
    perror("listen");
    exit(EXIT_FAILURE);
  }
  if ((new_socket = accept(server_fd, (struct sockaddr*)&address,
                           (socklen_t*)&addrlen)) < 0)
  {
    perror("accept");
    exit(EXIT_FAILURE);
  }

  while (true)
  {
    memset(recv_buffer, 0, 1024);
    memset(send_buffer, 0, 1024);

    std::cout << "Waiting for the input" << std::endl;
    std::cin.ignore();
    std::cout << "Input detected. Proceeding to the next step" << std::endl;

    recv(new_socket, recv_buffer, 1024, 0);
    std::cout << recv_buffer << std::endl;
    if (std::stoi(recv_buffer) == 1)
    {
      /// Status Ready -> Send coordinates
      std::cout << "Entered" << std::endl;
      getCoordinates();
      std::string testCoordintaes= "1,1,1,1,1,1";
      strcpy(send_buffer, testCoordintaes.c_str());
      send(new_socket, send_buffer, 1024, 0);
      memset(recv_buffer, 0, 1024);
      memset(send_buffer, 0, 1024);
    }
    else
    {
      //Possibly shutdown request?
      printf("The arm is not ready.");
    }
    sleep(1);
  }
  close(server_fd);
  shutdown(server_fd, SHUT_WR);
  return 0;
}
