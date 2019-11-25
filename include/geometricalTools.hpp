#ifndef GEOMETRICALTOOLS_HPP
#define GEOMETRICALTOOLS_HPP
#include <opencv2/core/types.hpp>
#include <opencv2/calib3d.hpp>

namespace arucol {

// Checks if a matrix is a valid rotation matrix.
inline bool isRotationMatrix(cv::Mat &R) {
  cv::Mat Rt;
  transpose(R, Rt);
  cv::Mat shouldBeIdentity = Rt * R;
  cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

  return cv::norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
inline cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R) {

  assert(isRotationMatrix(R));

  float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
                  R.at<double>(1, 0) * R.at<double>(1, 0));

  bool singular = sy < 1e-6; // If

  float x, y, z;
  if (!singular) {
    x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    y = atan2(-R.at<double>(2, 0), sy);
    z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
  } else {
    x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
    y = atan2(-R.at<double>(2, 0), sy);
    z = 0;
  }
  return cv::Vec3f(x, y, z);
}

inline void tvecAndRvecToHomogeneousMatrix(const cv::Vec3d &tvec,
                                           const cv::Vec3d &rvec, cv::Matx44d &homogeneousMatrix) {
  cv::Matx33d rotMat;
  cv::Rodrigues(rvec, rotMat);
  homogeneousMatrix = cv::Matx44d(
      rotMat(0, 0), rotMat(0, 1), rotMat(0, 2), tvec(0), 
      rotMat(1, 0), rotMat(1, 1), rotMat(1, 2), tvec(1),
      rotMat(2, 0), rotMat(2, 1), rotMat(2, 2), tvec(2), 
      0.0, 0.0, 0.0, 1.0);
}

inline void homogeneousMatrixToTvecAndRvec(const cv::Matx44d& homogeneousMatrix, cv::Vec3d& tvec, cv::Vec3d& rvec){
  cv::Matx33d rotMat(
    homogeneousMatrix(0, 0), homogeneousMatrix(0, 1), homogeneousMatrix(0, 2),
    homogeneousMatrix(1, 0), homogeneousMatrix(1, 1), homogeneousMatrix(1, 2),
    homogeneousMatrix(2, 0), homogeneousMatrix(2, 1), homogeneousMatrix(2, 2));
  cv::Rodrigues(rotMat, rvec);
  tvec = cv::Vec3d(homogeneousMatrix(0, 3), homogeneousMatrix(1, 3), homogeneousMatrix(2, 3));
}

inline void homogeneousMatrixFromTranslationRotation2D(const double x, const double y, const double orientation, cv::Matx44d& homogeneousMatrix){
  homogeneousMatrix = cv::Matx44d(
    cos(orientation), -sin(orientation), 0, x,
    sin(orientation), cos(orientation), 0, y,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0
  );
}


} // namespace arucol
#endif /* GEOMETRICALTOOLS_HPP */
