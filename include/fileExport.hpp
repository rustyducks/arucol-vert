#ifndef FILEEXPORT_HPP
#define FILEEXPORT_HPP

#include <string>
#include <unordered_map>
#include <opencv2/aruco.hpp>



namespace arucol{

// id,X,Y,Z;id,X,Y,Z;...

typedef std::unordered_map<int, cv::Matx44d> MarkerPoses_t;


class FileExport{
public:
    FileExport(const std::string& filename);

    void addTimeStep(const MarkerPoses_t& poses);
    void addTimeStep(const cv::Matx44d& pose);
    void addTimeStep();

    void write() const;

protected:
    std::string serialize(const MarkerPoses_t& poses) const;

    std::string filename_;
    std::vector<MarkerPoses_t> timesteps_;
};
}

#endif /* FILEEXPORT_HPP */
