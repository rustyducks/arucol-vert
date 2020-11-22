#include "UARTCommunication.hpp"

#include <cstring>
#include <iostream>

#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART

namespace arucol{

UARTCommunication::UARTCommunication(const std::string& uartFilestream): filestreamName_(uartFilestream), filestream_(-1){
    filestream_ = open(filestreamName_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (filestream_ == -1){
        //ERROR - CAN'T OPEN SERIAL PORT
        printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
    }
    struct termios options;
    tcgetattr(filestream_, &options);
    options.c_cflag = B115200 | CS8 | PARENB | CLOCAL | CREAD;		//<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(filestream_, TCIFLUSH);
    tcsetattr(filestream_, TCSANOW, &options);
}

UARTCommunication::~UARTCommunication(){
  if (filestream_ > 0){
    close(filestream_);
  }
}

void UARTCommunication::sendPose(float x, float y, float theta){
    ArucolPose msg;
    msg.x = x;
    msg.y = y;
    msg.theta = theta;

    uint8_t buf[SIZE_ArucolPose];
    arucolPoseToBytes(&msg, buf);
    write(filestream_, buf, sizeof(buf));
}

uint16_t UARTCommunication::computeChecksum(const uint8_t *buffer, int len) const {
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
  uint16_t checksum = computeChecksum(buffer+2, SIZE_ArucolPose - 4);
  buffer[offset++] = checksum & 0XFF;
  buffer[offset++] = (checksum>>8) & 0XFF;
}

}

