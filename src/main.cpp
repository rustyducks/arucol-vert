#include <iostream>
#include <sstream>

#include "arucolVert.hpp"

int main(int argc, char** argv) {

    bool withDisplay = false;
    if (argc < 3){
        arucol::ArucolVert::help();
        return -1;
    }else if (argc == 4){
        withDisplay = (bool)argv[3];
    }
    int cameraId;
    std::stringstream(argv[1]) >> cameraId;
    arucol::ArucolVert av(cameraId, argv[2], withDisplay);
    av.run();

/*     cv::VideoCapture inputVideo;
    inputVideo.open(0);
    cv::Ptr<cv::aruco::Dictionary> dictionnary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);

    cv::Mat cameraMatrix, distCoeffs;
// camera parameters are read from somewhere
    cv::FileStorage fs("out_camera_data.xml", cv::FileStorage::READ); // Read the camera settings 
    if (!fs.isOpened())
    {
        std::cout << "Could not open the configuration file" << std::endl;
        return -1;
    }
    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Distortion_Coefficients"] >> distCoeffs;
    fs.release();                                         // close Settings file
    std::cout << cameraMatrix << std::endl;
    std::cout << distCoeffs << std::endl;
    while (inputVideo.grab())
    {
        cv::Mat image, imageCopy;
        inputVideo.retrieve(image);
        image.copyTo(imageCopy);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionnary, corners, ids);

        if (ids.size() > 0){
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 0.068, cameraMatrix, distCoeffs, rvecs, tvecs);
            // draw axis for each marker
            for(int i=0; i<ids.size(); i++){
                std::cout << tvecs[i] << ";";
                cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
            }
            std::cout << std::endl;
        }

        cv::imshow("out", imageCopy);
        char key = (char) cv::waitKey(10);
        if (key == 27){
            break;
        }
    } */
    
}
