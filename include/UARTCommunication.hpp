#ifndef UARTCOMMUNICATION_HPP
#define UARTCOMMUNICATION_HPP
#include <string>

#define MAX_MSG_BUFFER_SIZE 18

#define SIZE_ArucolPose 18
#define  ID_ArucolPose 1

namespace arucol{
struct ArucolPose{
  float theta;
  float x;
  float y;
};



union Message_t {
  struct ArucolPose arucol_pose;
};

class UARTCommunication{
  public:
  UARTCommunication(const std::string& uartFilestream);
  ~UARTCommunication();

  void sendPose(float x, float y, float theta);
  protected:
  uint16_t computeChecksum(const uint8_t *buffer, const int len) const;

  void arucolPoseToBytes(const struct ArucolPose* msg, uint8_t *buffer) const;

  protected:
  std::string filestreamName_;
  int filestream_;

  
};
}
#endif /* UARTCOMMUNICATION_HPP */
