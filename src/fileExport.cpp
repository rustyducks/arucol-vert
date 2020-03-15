#include <fileExport.hpp>

#include <fstream>
#include <iostream>

#include <geometricalTools.hpp>


using namespace arucol;

FileExport::FileExport(const std::string& filename) : filename_(filename){
}

void FileExport::addTimeStep(const MarkerPoses_t& poses){
    timesteps_.push_back(poses);
}

void FileExport::addTimeStep(const cv::Matx44d& pose){
    MarkerPoses_t p;
    p[0] = pose;
    timesteps_.push_back(p);
}

void FileExport::addTimeStep(){
    timesteps_.push_back({});
}

void FileExport::write() const{
  std::ofstream file;
  file.open (filename_);
  for (const auto& step: timesteps_){
      file << serialize(step) << std::endl;
  }
  file.close();
}

std::string FileExport::serialize(const MarkerPoses_t& poses) const{
    std::stringstream stream;
    for (const auto& p: poses){
        cv::Vec3d tvec, rvec;
        std::cout << p.second << std::endl;
        homogeneousMatrixToTvecAndRvec(p.second, tvec, rvec);
        stream << p.first << "," << tvec[0] << "," << tvec[1] << "," << tvec[2] << ";";
    }
    std::string s = stream.str();
    if (s.size() > 0){
        s.pop_back();  // Remove the last ";" if any
    }
    return s;
}