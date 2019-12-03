#include <gtest/gtest.h>

#include "arucolVertParams.hpp"
#include "cameraParams.hpp"



class CameraParamTest : public ::testing::Test {
 protected:
  CameraParamTest() : ::testing::Test(), camParams("../test/data/test_camera_params.xml"){}  
  void SetUp() override {
  }

  arucol::CameraParams camParams;
};

TEST_F(CameraParamTest, ReadImageSize){
    ASSERT_EQ(camParams.size.width, 1296);
    ASSERT_EQ(camParams.size.height, 976);
}

TEST_F(CameraParamTest, CameraMatrix){
    ASSERT_EQ(camParams.matrix.cols, 3);
    ASSERT_EQ(camParams.matrix.rows, 3);
    ASSERT_EQ(camParams.matrix.at<double>(0, 0), 581.16599925576509);
    ASSERT_EQ(camParams.matrix.at<double>(0, 1), 0.0);
    ASSERT_EQ(camParams.matrix.at<double>(0, 2), 648.0);
    ASSERT_EQ(camParams.matrix.at<double>(1, 0), 0.0);
    ASSERT_EQ(camParams.matrix.at<double>(1, 1), 581.16599925576509);
    ASSERT_EQ(camParams.matrix.at<double>(1, 2), 488.);
    ASSERT_EQ(camParams.matrix.at<double>(2, 0), 0.);
    ASSERT_EQ(camParams.matrix.at<double>(2, 1), 0.);
    ASSERT_EQ(camParams.matrix.at<double>(2, 2), 1.);
}

TEST_F(CameraParamTest, DistortionCoeffs){
    ASSERT_EQ(camParams.distortionCoeffs.cols, 1);
    ASSERT_EQ(camParams.distortionCoeffs.rows, 5);
    ASSERT_EQ(camParams.distortionCoeffs.at<double>(0, 0), -2.5794892943718595e-01);
    ASSERT_EQ(camParams.distortionCoeffs.at<double>(1, 0), 5.8519678462648272e-02);
    ASSERT_EQ(camParams.distortionCoeffs.at<double>(2, 0), 0.0);
    ASSERT_EQ(camParams.distortionCoeffs.at<double>(3, 0), 0.0);
    ASSERT_EQ(camParams.distortionCoeffs.at<double>(4, 0), -5.4724218363248478e-03);
}
