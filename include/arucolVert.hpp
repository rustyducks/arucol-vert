#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <unordered_set>

#include "cameraParams.hpp"

namespace arucol {

enum eState { IDLE, AQCUIRING_CENTRAL_MARKER, RUNNING };

struct sPose {
  cv::Vec3d tvec;
  cv::Vec3d rvec;
};

struct sMarkers{
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
};

class ArucolVert {
public:
  ArucolVert(const unsigned int cameraId,
             const std::string &camaraParametersFilename,
             const bool withDisplay = false);
  ~ArucolVert();

  void run();

  static void help();

protected:
    void acquireCentralMarker();
    sMarkers filterMarkers(const sMarkers& markers, const std::unordered_set<int>& whitelistIds) const;

protected:
  eState state;
  CameraParams cameraParams;
  bool withDisplay;
  cv::VideoCapture inputVideo;
  cv::Ptr<cv::aruco::Dictionary> dictionnary;
  sPose centralMarkerPose;
};

} // namespace arucol
