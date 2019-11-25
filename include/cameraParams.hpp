#ifndef CAMERAPARAMS_HPP
#define CAMERAPARAMS_HPP
#include <opencv2/core/core.hpp>
#include <string>

namespace arucol {

class CameraParams {
public:
  CameraParams(const std::string &filename);

public:
  cv::Size size;
  cv::Mat matrix;
  cv::Mat distortionCoeffs;
}; // class CameraParams

} // namespace arucol
#endif /* CAMERAPARAMS_HPP */
