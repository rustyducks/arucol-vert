#ifndef ARUCOLVERTPARAMS_HPP
#define ARUCOLVERTPARAMS_HPP
#include <opencv2/core/core.hpp>
#include <unordered_set>
#include <unordered_map>

namespace arucol {

struct sMarkerParams{
  int id;
  double size;
  cv::Matx44d markerToRobot;
};

struct sHeightFilterParams{
  double minHeight;
  double maxHeight;
};

struct sRotationFilterParams{
  double nonVerticalAngleTolerance;
};

struct sUARTCommParams{
  std::string filename;
};

class ArucolVertParams {
public:
  ArucolVertParams(const std::string &filename);

public:
  enum eVideoInput { CAMERA, VIDEO_FILE };

  eVideoInput inputType;
  unsigned int cameraId;
  std::string videoFileName;


  sMarkerParams centralMarkerParams;  // central marker has markertoref stored in markerToRobot...
  int numberOfCentralMarkerDetectionBeforeStart;
  int averageFilterWeight;
  bool updateCentralMarkerWhileRunning;

  std::unordered_map<int, sMarkerParams> markersParams;
  std::unordered_set<int> whitelistedMarkers;

  sHeightFilterParams heightFilterParams;
  sRotationFilterParams rotationFilterParams;

  sUARTCommParams uartCommParams;



  double period; // In ms
};
} // namespace arucol
#endif /* ARUCOLVERTPARAMS_HPP */
