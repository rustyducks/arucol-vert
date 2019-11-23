#include "cameraParams.hpp"
#include <iostream>

namespace arucol {

CameraParams::CameraParams(const std::string &filename) {
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "Could not open the camera parameter file : \"" << filename
              << "\"." << std::endl;
  }
  fs["image_Width"] >> size.width;
  fs["image_Height"] >> size.height;
  fs["Camera_Matrix"] >> matrix;
  fs["Distortion_Coefficients"] >> distortionCoeffs;
  fs.release();
}

} // namespace arucol