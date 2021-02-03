#ifndef CAMERA_H
#define CAMERA_H

#include "Util.h"
#include "Frame.h"

#include <string>
#include <vector>

namespace cat {

namespace kf {

class Camera {
public:
    Camera(const std::string datasetDirectory, const std::string camIntrinsicsFileName);
    
    Frame getFrame();
    bool canGetFrame();

private:
    void setupFrames();
    
    int frameIndex;
    std::string datasetDirectory;

    std::vector<double> timestampsDepth;
    std::vector<double> timestampsColor;
    std::vector<std::string> fileNamesDepth;
    std::vector<std::string> fileNamesColor;

    CameraIntrinsics camIntrinsics;
    cv::Mat depthMap;
    cv::Mat colorMap;
};

} // namespace kf
    
} // namespace cat

#endif