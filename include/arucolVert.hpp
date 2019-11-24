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
  sMarkers filterMarkers(const sMarkers &markers,
                         const std::unordered_set<int> &whitelistIds) const;
  size_t findPoses(const cv::Mat &image, cv::Mat &debugImage,
                   std::unordered_map<int, cv::Matx44d> &poses) const;

protected:
  eState state;
  CameraParams cameraParams;
  ArucolVertParams params;
  bool withDisplay;
  cv::VideoCapture inputVideo;
  cv::Ptr<cv::aruco::Dictionary> dictionnary;
  cv::Matx44d centralMarkerPose;
  cv::Matx44d centralMarkerPoseInv;
};

} // namespace arucol
