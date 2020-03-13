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
  fs["centralMarkerId"] >> centralMarkerParams.id;
  fs["centralMarkerSize"] >> centralMarkerParams.size;
  std::vector<double> centralMarkerPosition;
  fs["centralMarkerPosition"] >> centralMarkerPosition;
  double centralMarkerOrientation;
  fs["centralMarkerOrientation"] >> centralMarkerOrientation;
  centralMarkerOrientation = centralMarkerOrientation * M_PI / 180.;
  homogeneousMatrixFromTranslationRotation2D(
      centralMarkerPosition[0], centralMarkerPosition[1],
      centralMarkerOrientation, centralMarkerParams.markerToRobot);
  fs["centralMarkerDetectionBeforeStart"] >> numberOfCentralMarkerDetectionBeforeStart;
  fs["averageFilterWeight"] >> averageFilterWeight;
  fs["updateCentralMarkerOnline"] >> updateCentralMarkerWhileRunning;

  cv::FileNode markers = fs["markers"];
  for (auto it = markers.begin(); it != markers.end(); ++it){
    sMarkerParams m;
    (*it)["id"] >> m.id;
    whitelistedMarkers.insert(m.id);
    (*it)["size"] >> m.size;
    std::vector<double> p;
    (*it)["markerToRobot"] >> p;
    cv::Vec3d tvec(p[0], p[1], p[2]);
    cv::Vec4d quaternion(p[3], p[4], p[5], p[6]);
    homogeneousMatrixFromTranslationQuaternion(tvec, quaternion, m.markerToRobot);
    markersParams[m.id] = m;
  }
  cv::FileNode markersBatch = fs["markersBatch"];
  for (auto it = markersBatch.begin(); it != markersBatch.end(); ++it){
    std::vector<int> ids;
    (*it)["ids"] >> ids;
    double size;
    (*it)["size"] >> size;
    std::vector<double> p;
    cv::Matx44d mat;
    (*it)["markerToRobot"] >> p;
    cv::Vec3d tvec(p[0], p[1], p[2]);
    cv::Vec4d quaternion(p[3], p[4], p[5], p[6]);
    homogeneousMatrixFromTranslationQuaternion(tvec, quaternion, mat);

    for (const auto& id: ids){
      whitelistedMarkers.insert(id);
      markersParams[id] = {id, size, mat};
    }

    fs["filters"]["heightFilter"]["minHeight"] >> heightFilterParams.minHeight;
    fs["filters"]["heightFilter"]["maxHeight"] >> heightFilterParams.maxHeight;

    fs["filters"]["rotationFilter"]["maxAxisDrift"] >> rotationFilterParams.nonVerticalTolerence;

  }



  /* validMarkerIds =
      std::unordered_set<int>(_validMarkersIds.begin(), _validMarkersIds.end());
  fs["markerSize"] >> markerSize;

  validSmallMarkerIds = {60, 61, 62, 63};
  smallMarkerSize = 0.05;  //TODO

  cv::Matx44d pose;
  tvecAndRvecToHomogeneousMatrix({0.0, -0.03,-0.04}, {0, -2.2214415, 2.2214415}, pose);
  markersToRobot[63] = pose;
  tvecAndRvecToHomogeneousMatrix({0.0, -0.03, -0.04}, {-2.4183992, -2.4183992, 2.4183992}, pose); // 61
  markersToRobot[61] = pose;
  tvecAndRvecToHomogeneousMatrix({0.03, 0.0, -0.04}, {2.2214415, 0, 2.2214415}, pose);  //60
  markersToRobot[60] = pose;
  tvecAndRvecToHomogeneousMatrix({0.0, 0.03, -0.04}, {0, 2.2214415, 2.2214415}, pose);  //62
  markersToRobot[62] = pose; */

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