#include "arucolVertParams.hpp"

#include <iostream>

#include "geometricalTools.hpp"

namespace arucol {
ArucolVertParams::ArucolVertParams(const std::string &filename) {
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "Could not open the general parameter file : \"" << filename
              << "\"." << std::endl;
  }
  std::string input;
  fs["cameraId"] >> input;
  if (input.empty()) {
    fs["videoFile"] >> input;
    if (input.empty()) {
      std::cout << "No video input provided. Please put either a \"cameraId\" "
                   "or \"videoFile\" tag in the "
                << filename << " config file." << std::endl;
    } else {
      inputType = VIDEO_FILE;
      videoFileName = input;
    }
  } else {
    inputType = CAMERA;
    std::stringstream ss(input);
    ss >> cameraId;
  }
  // cameraId = (unsigned int) _cameraId;
  fs["centralMarkerId"] >> centralMarkerId;
  fs["centralMarkerSize"] >> centralMarkerSize;
  std::vector<double> centralMarkerPosition;
  fs["centralMarkerPosition"] >> centralMarkerPosition;
  double centralMarkerOrientation;
  fs["centralMarkerOrientation"] >> centralMarkerOrientation;
  centralMarkerOrientation = centralMarkerOrientation * M_PI / 180.;
  homogeneousMatrixFromTranslationRotation2D(
      centralMarkerPosition[0], centralMarkerPosition[1],
      centralMarkerOrientation, refToCentralMarker);

  std::vector<int> _validMarkersIds;
  fs["markerIds"] >> _validMarkersIds;
  validMarkerIds =
      std::unordered_set<int>(_validMarkersIds.begin(), _validMarkersIds.end());
  fs["markerSize"] >> markerSize;

  double rate;
  fs["rate"] >> rate;
  if (rate == 0) {
    period = 0.0;
  } else {
    period = 1. / rate * 1000;
  }

  fs.release();
}
} // namespace arucol