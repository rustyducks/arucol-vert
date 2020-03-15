#ifndef ARUCOLVERT_HPP
#define ARUCOLVERT_HPP
#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "arucolVertParams.hpp"
#include "cameraParams.hpp"

namespace arucol {

enum eState { IDLE, AQCUIRING_CENTRAL_MARKER, RUNNING };

struct sPose {
  cv::Vec3d tvec;
  cv::Vec3d rvec;
};

struct sMarkers {
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
};

typedef std::unordered_map<int, cv::Matx44d> MarkerPoses_t;

class ArucolVert {
public:
  ArucolVert(const std::string &camaraParametersFilename,
             const std::string &parametersFilename,
             const bool withDisplay = false);
  ~ArucolVert();

  void run();

  static void help();

protected:
  bool updateCentralMarker(const cv::Mat &image, cv::Mat &debugImage);

  // Filters a list of markers, returns only those in "whitelistIds"
  sMarkers filterMarkers(const sMarkers &markers,
                         const std::unordered_set<int> &whitelistIds) const;

  std::unordered_map<double, sMarkers> groupMarkersBySize(const sMarkers &markers) const;
  size_t findPoses(const cv::Mat &image, cv::Mat &debugImage,
                   MarkerPoses_t &poses) const;

  size_t filterOnHeight(const MarkerPoses_t &markers, MarkerPoses_t &filteredMarkers) const;
  size_t filterOnRotation(const MarkerPoses_t &markers, MarkerPoses_t &filteredMarkers) const;

  void averageMarkerPoses(const MarkerPoses_t &markers, cv::Matx44d& pose) const;

  void drawFrame(cv::Mat& image, const cv::Matx44d& pose) const;

protected:
  eState state;
  CameraParams cameraParams;
  ArucolVertParams params;
  bool withDisplay;
  cv::VideoCapture inputVideo;
  cv::Ptr<cv::aruco::Dictionary> dictionnary;
  cv::Matx44d centralMarkerPose;
  cv::Matx44d centralMarkerPoseInv;
  cv::Matx44d refToCamera;
};

} // namespace arucol

#endif /* ARUCOLVERT_HPP */
