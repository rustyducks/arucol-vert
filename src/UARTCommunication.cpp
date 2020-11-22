#include "UARTCommunication.hpp"

#include <cstring>
#include <iostream>

#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART

UARTCommunication::UARTCommunication(const std::string& uartFilestream): filestreamName_(uartFilestream), filestream_(-1){
    filestream_ = open(filestreamName_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    		
}

void UARTCommunication::sendPose(double x, double y, double theta){
    ArucolPose msg;
    msg.x = x;
    msg.y = y;
    msg.theta = theta;

    uint8_t buf[SIZE_ArucolPose];
    arucolPoseToBytes(&msg, buf);

    std::cout << 
}

uint16_t UARTCommunication::computeCheksum(const uint8_t *buffer, int len) const {
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  for(int i=0; i<len; i++) {
    ck_a = (ck_a + buffer[i]);       // % 256 by overflow
    ck_b = (ck_b + ck_a);    // % 256 by overflow
  }
  uint16_t ck = (ck_a << 8) | ck_b;
  return ck;
}

void UARTCommunication::arucolPoseToBytes(const struct ArucolPose* msg, uint8_t *buffer) const{
  int offset = 0;
  buffer[offset++] = 0xFF;
  buffer[offset++] = 0xFF;
  buffer[offset++] = ID_ArucolPose;
  buffer[offset++] = SIZE_ArucolPose - 4;
  memcpy(buffer+offset, &msg->theta, 4);
  offset += 4;
  memcpy(buffer+offset, &msg->x, 4);
  offset += 4;
  memcpy(buffer+offset, &msg->y, 4);
  offset += 4;
  uint16_t checksum = computeCheksum(buffer+2, SIZE_ArucolPose - 4);
  buffer[offset++] = checksum & 0XFF;
  buffer[offset++] = (checksum>>8) & 0XFF;
}

void UARTCommunication::arucolPoseFromBytes(union Message_t* msg_u, const uint8_t *buffer) const{
  struct ArucolPose* msg = (struct ArucolPose*)msg_u;
  int offset = 0;
  memcpy(&msg->theta, buffer+offset, 4);
  offset += 4;
  memcpy(&msg->x, buffer+offset, 4);
  offset += 4;
  memcpy(&msg->y, buffer+offset, 4);
  offset += 4;
}

