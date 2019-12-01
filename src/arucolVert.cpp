#include "arucolVert.hpp"
#include <chrono>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>
#include <sstream>

#include "geometricalTools.hpp"

namespace arucol {

ArucolVert::ArucolVert(const std::string &cameraParametersFilename,
                       const std::string &parametersFilename,
                       const bool withDisplay)
    : state(IDLE), cameraParams(cameraParametersFilename),
      params(parametersFilename), withDisplay(withDisplay),
      dictionnary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100)) {

  if (params.inputType == ArucolVertParams::CAMERA) {
    inputVideo.open(params.cameraId);
  } else {
    inputVideo.open(params.videoFileName);
  }
  if (!inputVideo.isOpened()) {
    std::cout << "Unable to open video input." << std::endl;
  }
}

ArucolVert::~ArucolVert() {}

void ArucolVert::help() {
  std::cout << "ArucolVert Aruco pose estimator executable." << std::endl
            << "Usage: ./arucol_vert CAMERA_PARAMETERS_FILE "
               "GENERAL_PARAMETERS_FILE [WITH_DISPLAY]"
            << std::endl;
}

void ArucolVert::run() {
  state = AQCUIRING_CENTRAL_MARKER;
  cv::Mat img, debugImg;
  std::cout << "Acquiring central marker, id: " << params.centralMarkerId
            << std::endl;
  while (inputVideo.grab()) {
    inputVideo.retrieve(img);
    if (withDisplay) {
      img.copyTo(debugImg);
    }
    if (updateCentralMarker(img, debugImg)) {
      break;
    }
  }

  std::cout << "Starting poses estimations" << std::endl;
  state = RUNNING;
  std::unordered_map<int, cv::Matx44d> markerPoses;
  std::chrono::time_point<std::chrono::system_clock> lastFrameTime, now;
  while (inputVideo.grab()) {
    now = std::chrono::high_resolution_clock::now();
    inputVideo.retrieve(img);
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                              lastFrameTime)
            .count() < params.period) {
      continue;
    }
    lastFrameTime = now;
    if (withDisplay) {
      img.copyTo(debugImg);
    }
    updateCentralMarker(img, debugImg);
    findPoses(img, debugImg, markerPoses);
    std::cout << markerPoses.size() << std::endl;
    for (auto &it : markerPoses) {
      cv::Vec3d rvec, tvec;
      homogeneousMatrixToTvecAndRvec(it.second, tvec, rvec);
      std::cout << "Marker : " << it.first << "@" << tvec << std::endl;
    }
    if (withDisplay) {
      cv::imshow("Output", debugImg);
      int pressedKey = cv::waitKey(10);
      if (pressedKey == 32){
        pressedKey = cv::waitKey(10);
        while (pressedKey != 32){
          pressedKey = cv::waitKey(100);
        }
      }
    }
  }
}

bool ArucolVert::updateCentralMarker(const cv::Mat &image,
                                     cv::Mat &debugImage) {
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(image, dictionnary, corners, ids);
  sMarkers centralMarker =
      filterMarkers({ids, corners}, {params.centralMarkerId});

  if (centralMarker.ids.size() == 1) {
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(
        centralMarker.corners, params.centralMarkerSize, cameraParams.matrix,
        cameraParams.distortionCoeffs, rvecs, tvecs);

    if (withDisplay) {
      cv::aruco::drawDetectedMarkers(debugImage, centralMarker.corners,
                                     centralMarker.ids);
      cv::aruco::drawAxis(debugImage, cameraParams.matrix,
                          cameraParams.distortionCoeffs, rvecs[0], tvecs[0],
                          0.1);
    }
    tvecAndRvecToHomogeneousMatrix(tvecs[0], rvecs[0], centralMarkerPose);
    centralMarkerPoseInv = centralMarkerPose.inv();
    refToCamera = params.refToCentralMarker * centralMarkerPoseInv;
    // std::cout << "Updating central marker pose: " << centralMarkerPose
    //          << std::endl;
    return true;
  } else if (centralMarker.ids.size() > 1) {
    std::cout << "Warning: Multiple central markers with id :"
              << params.centralMarkerId
              << "found on the same frame. Not updating the pose" << std::endl;
    return false;
  }
  return false;
}

sMarkers
ArucolVert::filterMarkers(const sMarkers &markers,
                          const std::unordered_set<int> &whitelistIds) const {
  sMarkers filtered;
  for (size_t i = 0; i < markers.ids.size(); i++) {
    if (whitelistIds.count(markers.ids[i]) == 1) {
      filtered.ids.push_back(markers.ids[i]);
      filtered.corners.push_back(markers.corners[i]);
    }
  }
  return filtered;
}

size_t
ArucolVert::findPoses(const cv::Mat &image, cv::Mat &debugImage,
                      std::unordered_map<int, cv::Matx44d> &poses) const {
  poses.clear();
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(image, dictionnary, corners, ids);
  sMarkers markers = filterMarkers({ids, corners}, params.validMarkerIds);
  if (markers.ids.size() == 0) {
    std::cout << "No marker found" << std::endl;
    return 0;
  }
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(
      markers.corners, params.markerSize, cameraParams.matrix,
      cameraParams.distortionCoeffs, rvecs, tvecs);

  std::vector<cv::Matx44d> camToMarkers;
  camToMarkers.reserve(markers.ids.size());
  for (size_t i = 0; i < markers.ids.size(); i++) {
    cv::Matx44d camToMarker;
    tvecAndRvecToHomogeneousMatrix(tvecs[i], rvecs[i], camToMarker);
    poses[markers.ids[i]] = refToCamera * camToMarker;
    camToMarkers.push_back(camToMarker);
  }

  if (withDisplay) {
    cv::aruco::drawDetectedMarkers(debugImage, markers.corners, markers.ids);
    if (markers.ids.size() > 0) {
      std::vector<cv::Point3d> points;
      std::vector<cv::Point2d> imgPoints;
      cv::Point3d centerPose;
      cv::Vec3d pose, rvec;
      homogeneousMatrixToTvecAndRvec(refToCamera.inv(), pose, rvec);
      centerPose.x = pose[0];
      centerPose.y = pose[1];
      centerPose.z = pose[2];
      points.push_back(centerPose);
      for (size_t i = 0; i < markers.ids.size(); i++) {
        homogeneousMatrixToTvecAndRvec(camToMarkers[i], pose, rvec);
        cv::Point3d p(pose[0], pose[1], pose[2]);
        points.push_back(p);
      }

      cv::projectPoints(points, std::vector<double>({0.0, 0.0, 0.0}),
                        std::vector<double>({0.0, 0.0, 0.0}),
                        cameraParams.matrix, cameraParams.distortionCoeffs,
                        imgPoints);

      for (size_t i = 1; i < imgPoints.size(); i++){
        cv::line(debugImage, imgPoints[0], imgPoints[i], {0, 0, 255}, 2);
        std::stringstream ss;
        homogeneousMatrixToTvecAndRvec(poses[markers.ids[i-1]], pose, rvec);
        ss << "x:" << pose[0] << " y:" << pose[1] << " z:" << pose[2];
        cv::putText(debugImage, ss.str(), (imgPoints[0] + imgPoints[i])/2, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 0, 255}, 2);
      }
    }
  }

  return poses.size();
}
} // namespace arucol