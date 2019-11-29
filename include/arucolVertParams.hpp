#ifndef ARUCOLVERTPARAMS_HPP
#define ARUCOLVERTPARAMS_HPP
#include <opencv2/core/core.hpp>
#include <unordered_set>

namespace arucol {

class ArucolVertParams {
public:
  ArucolVertParams(const std::string &filename);

public:
  enum eVideoInput { CAMERA, VIDEO_FILE };

  eVideoInput inputType;
  unsigned int cameraId;
  std::string videoFileName;

  int centralMarkerId;
  double centralMarkerSize;

  std::unordered_set<int> validMarkerIds;
  double markerSize;

  double period; // In ms

  cv::Matx44d refToCentralMarker;
};
} // namespace arucol
#endif /* ARUCOLVERTPARAMS_HPP */
