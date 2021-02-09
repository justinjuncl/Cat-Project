#ifndef FRAME_H
#define FRAME_H

#include "Util.h"

namespace cat {

namespace kf {

struct Frame {
    cv::Mat depthMap;
    cv::Mat colorMap;

    cv::Mat filteredDepthMap;

    cv::Mat vertexMap;
    cv::Mat normalMap;

    cv::Affine3f pose;
    CameraIntrinsics camIntrinsics;

    Frame(const cv::Mat& depthMap, const cv::Mat& colorMap,
          const CameraIntrinsics& camIntrinsics)
        : depthMap(depthMap), colorMap(colorMap),
          camIntrinsics(camIntrinsics) {
    };
};

} // namespace kf
    
} // namespace cat

#endif