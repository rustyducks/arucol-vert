#include "arucolVertParams.hpp"

#include <iostream>

namespace arucol {
ArucolVertParams::ArucolVertParams(const std::string &filename) {
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "Could not open the general parameter file : \"" << filename
              << "\"." << std::endl;
  }
  int _cameraId;
  fs["cameraId"] >> _cameraId;
  cameraId = (unsigned int) _cameraId;
  fs["centralMarkerId"] >> centralMarkerId;
  fs["centralMarkerSize"] >> centralMarkerSize;
  std::vector<int> _validMarkersIds;
  fs["markerIds"] >> _validMarkersIds;
  validMarkerIds = std::unordered_set<int>(_validMarkersIds.begin(), _validMarkersIds.end());
  fs["markerSize"] >> markerSize;
  double rate;
  fs["rate"] >> rate;
  period = 1. / rate * 1000;
  fs.release();
}
}          