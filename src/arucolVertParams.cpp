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
  int _cameraId;
  fs["cameraId"] >> _cameraId;
  cameraId = (unsigned int) _cameraId;
  fs["centralMarkerId"] >> centralMarkerId;
  fs["centralMarkerSize"] >> centralMarkerSize;
  std::vector<double> centralMarkerPosition;
  fs["centralMarkerPosition"] >> centralMarkerPosition;
  double centralMarkerOrientation;
  fs["centralMarkerOrientation"] >> centralMarkerOrientation;
  centralMarkerOrientation = centralMarkerOrientation * M_PI / 180.;
  homogeneousMatrixFromTranslationRotation2D(centralMarkerPosition[0], centralMarkerPosition[1], centralMarkerOrientation, refToCentralMarker);

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