#ifndef GEOMETRICALTOOLS_HPP
#define GEOMETRICALTOOLS_HPP
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>

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
                                           const cv::Vec3d &rvec,
                                           cv::Matx44d &homogeneousMatrix) {
  cv::Matx33d rotMat;
  cv::Rodrigues(rvec, rotMat);
  homogeneousMatrix = cv::Matx44d(
      rotMat(0, 0), rotMat(0, 1), rotMat(0, 2), tvec(0), rotMat(1, 0),
      rotMat(1, 1), rotMat(1, 2), tvec(1), rotMat(2, 0), rotMat(2, 1),
      rotMat(2, 2), tvec(2), 0.0, 0.0, 0.0, 1.0);
}

inline void homogeneousMatrixToTvecAndRvec(const cv::Matx44d &homogeneousMatrix,
                                           cv::Vec3d &tvec, cv::Vec3d &rvec) {
  cv::Matx33d rotMat(homogeneousMatrix(0, 0), homogeneousMatrix(0, 1),
                     homogeneousMatrix(0, 2), homogeneousMatrix(1, 0),
                     homogeneousMatrix(1, 1), homogeneousMatrix(1, 2),
                     homogeneousMatrix(2, 0), homogeneousMatrix(2, 1),
                     homogeneousMatrix(2, 2));
  cv::Rodrigues(rotMat, rvec);
  tvec = cv::Vec3d(homogeneousMatrix(0, 3), homogeneousMatrix(1, 3),
                   homogeneousMatrix(2, 3));
}

inline void
homogeneousMatrixFromTranslationRotation2D(const double x, const double y,
                                           const double orientation,
                                           cv::Matx44d &homogeneousMatrix) {
  homogeneousMatrix = cv::Matx44d(cos(orientation), -sin(orientation), 0, x,
                                  sin(orientation), cos(orientation), 0, y, 0.0,
                                  0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
}

inline void averageHomogeneousMatrices(const cv::Matx44d &mat1, const cv::Matx44d &mat2, cv::Matx44d &result, const int mat1Weight = 1){
  cv::Vec3d tvec1, tvec2, rvec1, rvec2;
  cv::Matx44d rot1, rot2, tmp;
  tmp = cv::Matx44d::eye();
  homogeneousMatrixToTvecAndRvec(mat1, tvec1, rvec1);
  homogeneousMatrixToTvecAndRvec(mat2, tvec2, rvec2);
  tvecAndRvecToHomogeneousMatrix({0, 0, 0}, rvec1, rot1);
  tvecAndRvecToHomogeneousMatrix({0, 0, 0}, rvec2, rot2);

  tvec1 = tvec1 * mat1Weight;
  rvec1 = rvec1 * mat1Weight;

  tvec1 = (tvec1 + tvec2) / (mat1Weight + 1);
  rvec1 = (rvec1 + rvec2) / (mat1Weight + 1);

  tvecAndRvecToHomogeneousMatrix(tvec1, rvec1, result);


  // Complicated method (not working...):
  /*for (int i = 0; i < mat1Weight; i++){
    tmp = tmp * rot1;
  }
  tvec1 = tvec1 * mat1Weight;
  

  cv::Vec3d tvecResult, rvecResult, tvectmp;
  cv::Matx44d rotResult, result_;
  tvecResult = tvec1 + tvec2;
  rotResult = tmp * rot2;
  homogeneousMatrixToTvecAndRvec(rotResult, tvectmp, rvecResult);
  tvecResult = tvecResult / (mat1Weight + 1);
  rvecResult = rvecResult / (mat1Weight + 1);
  std::cout << rvecResult << std::endl;*/
  /*double angle;
  rvecToAxisAngle(rvec, axis, angle);
  angle = angle / (mat1Weight + 1);
  axisAngleToRvec(axis, angle, rvec);*/
  //tvecAndRvecToHomogeneousMatrix(tvecResult, rvecResult, result);

}

inline void rvecToAxisAngle(const cv::Vec3d& rvec, cv::Vec3d& axis, double& angle){
  angle = cv::norm(rvec);
  axis = cv::normalize(rvec);
}

inline void axisAngleToRvec(const cv::Vec3d& axis, const double& angle, cv::Vec3d& rvec){
  rvec = cv::normalize(axis);
  rvec = angle * rvec;
}

} // namespace arucol
#endif /* GEOMETRICALTOOLS_HPP */
