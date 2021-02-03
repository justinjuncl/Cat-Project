#ifndef FRAME_H
#define FRAME_H

#include "Util.h"

namespace cat {

namespace kf {

class Frame {
public:
    Frame(const cv::Mat& depthMap, const cv::Mat& colorMap, CameraIntrinsics& camIntrinsics);
    
public:
    cv::Mat depthMap;
    cv::Mat colorMap;

    cv::Mat filteredDepthMap;

    cv::Mat vertexMap;
    cv::Mat normalMap;

    cv::Affine3f pose;
    CameraIntrinsics camIntrinsics;
};

} // namespace kf
    
} // namespace cat

#endif