#include "arucolVert.hpp"
#include <chrono>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>
#include <sstream>

#include "geometricalTools.hpp"
#include "fileExport.hpp"
#include "UARTCommunication.hpp"

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
  std::cout << "Acquiring central marker, id: " << params.centralMarkerParams.id
            << std::endl;
  int i = 0;
  while (inputVideo.grab()) {
    inputVideo.retrieve(img);
    if (withDisplay) {
      img.copyTo(debugImg);
    }
    if (updateCentralMarker(img, debugImg)) {
      i++;
      if (i > params.numberOfCentralMarkerDetectionBeforeStart){
        break;
      }
    }
  }

  std::cout << "Starting poses estimations" << std::endl;
  state = RUNNING;
  std::unordered_map<int, cv::Matx44d> markerPoses;
  std::chrono::time_point<std::chrono::system_clock> lastFrameTime, now;
  UARTCommunication uartComm(params.uartCommParams.filename);
  cv::Vec3d robotTVec, robotRVec, robotEuler;

  FileExport fe("poses.csv");
  
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
    if (params.updateCentralMarkerWhileRunning){
      updateCentralMarker(img, debugImg);
    }
    findPoses(img, debugImg, markerPoses);
    MarkerPoses_t heightFiltered, rotFiltered;
    filterOnRotation(markerPoses, heightFiltered);
    markerPoses = heightFiltered;
    std::cout << markerPoses.size() << std::endl;
    for (auto &it : markerPoses) {
      cv::Vec3d rvec, tvec;
      homogeneousMatrixToTvecAndRvec(it.second, tvec, rvec);
      std::cout << "Marker : " << it.first << "@" << tvec << std::endl;
    }
    if (markerPoses.size() > 0){
      cv::Matx44d robotPose;
      averageMarkerPoses(markerPoses, robotPose);
      homogeneousMatrixToTvecAndRvec(robotPose, robotTVec, robotRVec);
      rvecToEuler(robotRVec, robotEuler);
      uartComm.sendPose(robotTVec[0], robotTVec[1], robotEuler[0]);
      fe.addTimeStep(robotPose);
    }else{
      fe.addTimeStep();
    }
    if (withDisplay) {
      drawFrame(debugImg, centralMarkerPose);
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

  fe.write();
}

bool ArucolVert::updateCentralMarker(const cv::Mat &image,
                                     cv::Mat &debugImage) {
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(image, dictionnary, corners, ids);
  sMarkers centralMarkers =
      filterMarkers({ids, corners}, {params.centralMarkerParams.id});

  if (centralMarkers.ids.size() == 1) {
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(
        centralMarkers.corners, params.centralMarkerParams.size, cameraParams.matrix,
        cameraParams.distortionCoeffs, rvecs, tvecs);

    if (withDisplay) {
      cv::aruco::drawDetectedMarkers(debugImage, centralMarkers.corners,
                                     centralMarkers.ids);
      cv::aruco::drawAxis(debugImage, cameraParams.matrix,
                          cameraParams.distortionCoeffs, rvecs[0], tvecs[0],
                          0.1);
    }
    cv::Matx44d tmp;
    tvecAndRvecToHomogeneousMatrix(tvecs[0], rvecs[0], tmp);
    if (centralMarkerPose == cv::Matx44d::zeros()){
      centralMarkerPose = tmp;
    }else{
      averageHomogeneousMatrices(centralMarkerPose, tmp, centralMarkerPose, params.averageFilterWeight);
    }

    centralMarkerPoseInv = centralMarkerPose.inv();
    refToCamera = params.centralMarkerParams.markerToRobot * centralMarkerPoseInv; // refToCentralMarker is stored in markerToRobot...

    // std::cout << "Updating central marker pose: " << centralMarkerPose
    //          << std::endl;
    return true;
  } else if (centralMarkers.ids.size() > 1) {
    std::cout << "Warning: Multiple central markers with id :"
              << params.centralMarkerParams.id
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

std::unordered_map<double, sMarkers> ArucolVert::groupMarkersBySize(const sMarkers &markers) const{
  std::unordered_map<double, sMarkers> groups;
  for (size_t i = 0; i < markers.ids.size(); i++){
    groups[params.markersParams.at(markers.ids[i]).size].ids.push_back(markers.ids[i]);
    groups[params.markersParams.at(markers.ids[i]).size].corners.push_back(markers.corners[i]);
  }
  return groups;
}

size_t
ArucolVert::findPoses(const cv::Mat &image, cv::Mat &debugImage,
                      MarkerPoses_t &poses) const {
  poses.clear();
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(image, dictionnary, corners, ids);
  sMarkers markers = filterMarkers({ids, corners}, params.whitelistedMarkers);
  size_t nMarkers = markers.ids.size();
  if (nMarkers == 0) {
    std::cout << "No marker found" << std::endl;
    return 0;
  }
  std::cout << "Found " << nMarkers << " filtered markers" << std::endl;
  std::unordered_map<double, sMarkers> groups = groupMarkersBySize(markers);



  std::vector<int> markersIds;
  std::vector<cv::Vec3d> rvecs, tvecs, locrvecs, loctvecs;
  for (const auto& p: groups){
    cv::aruco::estimatePoseSingleMarkers(p.second.corners, p.first, cameraParams.matrix, cameraParams.distortionCoeffs, locrvecs, loctvecs);
    rvecs.insert(rvecs.end(), locrvecs.begin(), locrvecs.end());
    tvecs.insert(tvecs.end(), loctvecs.begin(), loctvecs.end());
    markersIds.insert(markersIds.end(), p.second.ids.begin(), p.second.ids.end());
  }

  assert(markersIds.size() == nMarkers);

  std::vector<cv::Matx44d> camToMarkers;
  camToMarkers.reserve(nMarkers);
  for (size_t i = 0; i < nMarkers; i++) {
    cv::Matx44d camToMarker;
    tvecAndRvecToHomogeneousMatrix(tvecs[i], rvecs[i], camToMarker);
    poses[markersIds[i]] = refToCamera * camToMarker * params.markersParams.at(markersIds[i]).markerToRobot;
    camToMarkers.push_back(camToMarker * params.markersParams.at(markersIds[i]).markerToRobot);
  }

  assert(poses.size() == nMarkers);

  if (withDisplay) {
    /*cv::aruco::drawDetectedMarkers(debugImage, markers.corners, markers.ids);
    cv::aruco::drawDetectedMarkers(debugImage, smallMarkers.corners, smallMarkers.ids);*/
    if (nMarkers > 0) {
      std::vector<cv::Point3d> points;
      std::vector<cv::Point2d> imgPoints;
      cv::Point3d centerPose;
      cv::Vec3d pose, rvec;
      homogeneousMatrixToTvecAndRvec(refToCamera.inv(), pose, rvec);
      centerPose.x = pose[0];
      centerPose.y = pose[1];
      centerPose.z = pose[2];
      points.push_back(centerPose);
      drawFrame(debugImage, refToCamera.inv());
      for (size_t i = 0; i < camToMarkers.size(); i++) {
        homogeneousMatrixToTvecAndRvec(camToMarkers[i], pose, rvec);
        cv::Point3d p(pose[0], pose[1], pose[2]);
        points.push_back(p);
        drawFrame(debugImage, camToMarkers[i]);
      }

      cv::projectPoints(points, std::vector<double>({0.0, 0.0, 0.0}),
                        std::vector<double>({0.0, 0.0, 0.0}),
                        cameraParams.matrix, cameraParams.distortionCoeffs,
                        imgPoints);

      for (size_t i = 1; i < imgPoints.size(); i++){
        cv::line(debugImage, imgPoints[0], imgPoints[i], {0, 0, 255}, 2);
        std::stringstream ss;
        homogeneousMatrixToTvecAndRvec(poses[markersIds[i-1]], pose, rvec);
        ss << "x:" << pose[0] << " y:" << pose[1] << " z:" << pose[2];
        cv::putText(debugImage, ss.str(), (imgPoints[0] + imgPoints[i])/2, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 0, 255}, 2);
      }
    }
  }

  return poses.size();
  return 0;
}

void ArucolVert::drawFrame(cv::Mat& image, const cv::Matx44d& pose) const{
  std::vector<cv::Point3d> ps = {{0, 0, 0}, {0.1, 0, 0}, {0, 0.1, 0}, {0, 0, 0.1}};
  cv::Vec3d tvec, rvec;
  homogeneousMatrixToTvecAndRvec(pose, tvec, rvec);
  std::vector<cv::Point2d> pps;
  cv::projectPoints(ps, rvec, tvec, cameraParams.matrix, cameraParams.distortionCoeffs, pps);
  cv::line(image, pps[0], pps[1], {0, 0, 255}, 5);
  cv::line(image, pps[0], pps[2], {0, 255, 0}, 5);
  cv::line(image, pps[0], pps[3], {255, 0, 0}, 5);
}

size_t ArucolVert::filterOnHeight(const MarkerPoses_t &markers, MarkerPoses_t &filteredMarkers) const{
  filteredMarkers.clear();
  for (const auto& p: markers){
    if (p.second(2,3) <= params.heightFilterParams.maxHeight && p.second(2,3) >= params.heightFilterParams.minHeight){
      filteredMarkers[p.first] = p.second;
    }
  }
  return filteredMarkers.size();
}
size_t ArucolVert::filterOnRotation(const MarkerPoses_t &markers, MarkerPoses_t &filteredMarkers) const{
  filteredMarkers.clear();
  cv::Vec3d rVec, tVec;
  cv::Vec4d z(0.0, 0.0, 1.0, 1.0), z0(0.0, 0.0, 1.0, 0.0);
  double angle;
  for (const auto& p: markers){
    cv::Matx44d rot = p.second;
    rot(0, 3) = 0.0;
    rot(1, 3) = 0.0;
    rot(2, 3) = 0.0;
    cv::Vec4d zTransform = rot * z;

    angle = acos(zTransform.dot(z0));

    if (angle <= params.rotationFilterParams.nonVerticalAngleTolerance){
      filteredMarkers[p.first] = p.second;
    }
    
  }
  return filteredMarkers.size();
}

void ArucolVert::averageMarkerPoses(const MarkerPoses_t &markers, cv::Matx44d& pose) const{
  cv::Matx44d tmp = cv::Matx44d::eye(), tmp2;
  int n = 0;
  for (const auto& p: markers){
    averageHomogeneousMatrices(tmp, p.second, tmp2, n++);
    tmp = tmp2;
    std::cout << tmp <<std::endl;
  }
  pose = tmp;
}

} // namespace arucol