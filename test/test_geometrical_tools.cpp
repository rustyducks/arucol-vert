#include <gtest/gtest.h>

#include "geometricalTools.hpp"

class GeometricalTools : public ::testing::Test {
protected:
  GeometricalTools() : ::testing::Test() {}
  void SetUp() override{};
};

TEST_F(GeometricalTools, AverageMatrices) {
  // Same Matrices
  cv::Matx44d mat1, mat2, mat3;
  arucol::tvecAndRvecToHomogeneousMatrix({2.0, 3.0, 4.0}, {1.0, 1.0, 1.0},
                                         mat1);
  arucol::tvecAndRvecToHomogeneousMatrix({2.0, 3.0, 4.0}, {1.0, 1.0, 1.0},
                                         mat2);
  arucol::averageHomogeneousMatrices(mat1, mat2, mat3);
  cv::Vec3d tvec, rvec;
  arucol::homogeneousMatrixToTvecAndRvec(mat3, tvec, rvec);
  ASSERT_EQ(tvec, cv::Vec3d(2.0, 3.0, 4.0));
  ASSERT_LT(cv::norm(rvec - cv::Vec3d(1.0, 1.0, 1.0)), 0.00001);

  // Different matrices
  arucol::tvecAndRvecToHomogeneousMatrix({2.0, 3.0, 4.0}, {1.0, 1.0, 1.0},
                                         mat1);
  arucol::tvecAndRvecToHomogeneousMatrix({1.0, 5.0, 14.0}, {1.2, 1.2, 1.2},
                                         mat2);

  arucol::averageHomogeneousMatrices(mat1, mat2, mat3);
  arucol::homogeneousMatrixToTvecAndRvec(mat3, tvec, rvec);

  ASSERT_LT(cv::norm(tvec - cv::Vec3d(1.5, 4.0, 9.0)), 0.00001);
  ASSERT_LT(cv::norm(rvec - cv::Vec3d(1.1, 1.1, 1.1)), 0.00001); 

}