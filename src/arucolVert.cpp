#include "arucolVert.hpp"
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "geometricalTools.hpp"

namespace arucol {

ArucolVert::ArucolVert(const unsigned int cameraId,
                       const std::string &cameraParametersFilename,
                       const bool withDisplay)
    : state(IDLE), cameraParams(cameraParametersFilename),
      withDisplay(withDisplay),
      dictionnary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100)) {

  inputVideo.open(cameraId);
}

ArucolVert::~ArucolVert() {}

void ArucolVert::help() {
  std::cout
      << "ArucolVert Aruco pose estimator executable." << std::endl
      << "Usage: ./arucol_vert CAMERA_ID CAMERA_PARAMETERS_FILE [WITH_DISPLAY]"
      << std::endl;
}

void ArucolVert::run() {
  state = AQCUIRING_CENTRAL_MARKER;
  acquireCentralMarker();
  state = RUNNING;
}

void ArucolVert::acquireCentralMarker() {
  std::cout << "Acquiring central marker, id : " << 42 << std::endl;
  while (inputVideo.grab()) {
    cv::Mat image, imageCopy;
    inputVideo.retrieve(image);
    if (withDisplay) {
      image.copyTo(imageCopy);
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionnary, corners, ids);
    sMarkers centralMarker = filterMarkers({ids, corners}, {42});

    if (centralMarker.ids.size() == 1){
      if (withDisplay) {
        cv::aruco::drawDetectedMarkers(imageCopy, centralMarker.corners, centralMarker.ids);
      }
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(centralMarker.corners, 0.100, cameraParams.matrix,
                                           cameraParams.distortionCoeffs, rvecs,
                                           tvecs);
      std::cout << rvecs[0] << tvecs[0] << std::endl;


      if (withDisplay) {
        cv::aruco::drawAxis(imageCopy, cameraParams.matrix,
                              cameraParams.distortionCoeffs, rvecs[0], tvecs[0],
                              0.1);
        cv::imshow("out", imageCopy);
        char key = (char)cv::waitKey(10);
        if (key == 27) {
          break;
        }
      }
      centralMarkerPose.rvec = rvecs[0];
      centralMarkerPose.tvec = tvecs[0];
      std::cout << "Central marker found at " << centralMarkerPose.tvec << " with rotation " << centralMarkerPose.rvec << std::endl;
      break;
    }else if (centralMarker.ids.size() > 1){
      std::cout << "Warning: Multiple central markers with id " << 42 << "found on the same frame. Retrying..." << std::endl;
    }
  }
}

sMarkers ArucolVert::filterMarkers(const sMarkers& markers, const std::unordered_set<int>& whitelistIds) const{
  sMarkers filtered;
  for (size_t i=0; i < markers.ids.size(); i++){
    if (whitelistIds.count(markers.ids[i]) == 1){
      filtered.ids.push_back(markers.ids[i]);
      filtered.corners.push_back(markers.corners[i]);
    }
  }
  return filtered;
}
} // namespace arucol