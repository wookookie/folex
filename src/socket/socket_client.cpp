/******************************************************************************
* Copyright 2022 Hyunwook Choi
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
******************************************************************************/

#include <iostream>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include "essential.hpp"
#include "socket_client.hpp"


SocketClient::SocketClient()
{}

SocketClient::~SocketClient()
{
  closeConnection();
}

void SocketClient::connectServer()
{
  struct sockaddr_in server_addr_;

  // Create socket
  if ((socketfd_recv_ = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    perror("[SOCKET] socket()");
    exit(EXIT_FAILURE);
  }
  if ((socketfd_send_ = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    perror("[SOCKET] socket()");
    exit(EXIT_FAILURE);
  }

  // Set address and connect to server
  bzero(&server_addr_, sizeof(server_addr_));
  server_addr_.sin_family = AF_INET;
  server_addr_.sin_addr.s_addr = inet_addr(TCP_HOSTNAME);

  // Data receive connection
  server_addr_.sin_port = htons(TCP_PORT_RECV);
  if (connect(socketfd_recv_, (struct sockaddr *)&server_addr_, sizeof(server_addr_)) < 0)
  {
    perror("[SOCKET] connect()");
    exit(EXIT_FAILURE);
  }
  // Data send connection
  server_addr_.sin_port = htons(TCP_PORT_SEND);
  if (connect(socketfd_send_, (struct sockaddr *)&server_addr_, sizeof(server_addr_)) < 0)
  {
    perror("[SOCKET] connect()");
    exit(EXIT_FAILURE);
  }
}

void SocketClient::closeConnection()
{
  close(socketfd_recv_);
  close(socketfd_send_);
}

void SocketClient::receiveData()
{
  char _recv_data[1024];
  while (true)
  {
    if (read(socketfd_recv_, _recv_data, sizeof(_recv_data)) <= 0)
    {
      perror("[SOCKET] read()");
      exit(EXIT_FAILURE);
    }
    std::cout << "Data: " << _recv_data << std::endl;

    // Clear buffer
    bzero(_recv_data, sizeof(_recv_data));
  }
}

void SocketClient::sendData()
{
  // Combine data
  std::string _data_str;
  _data_str = std::to_string(Joint::present_angle_value[0]) + " " + std::to_string(Joint::present_angle_value[1]) + " " + std::to_string(Joint::present_angle_value[2]) + " " +
              std::to_string(Joint::present_angle_value[3]) + " " + std::to_string(Joint::present_angle_value[4]) + " " + std::to_string(Joint::present_angle_value[5]) + " " +
              std::to_string(Joint::present_angle_value[6]) + " " + std::to_string(Joint::present_angle_value[7]) + " " + std::to_string(Joint::present_angle_value[8]) + " " +
              std::to_string(Joint::present_angle_value[9]) + " " + std::to_string(Joint::present_angle_value[10]) + " " + std::to_string(Joint::present_angle_value[11]);

  // Convert string to charactor
  uint8_t *_data_char = new uint8_t[_data_str.size() + 1];
  std::copy(_data_str.begin(), _data_str.end(), _data_char);
  _data_char[_data_str.size()] = '\0';

  write(socketfd_send_, _data_char, _data_str.size() + 1);
  delete[] _data_char;
}
