#include "Camera.h"

#include <cstdio>

namespace cat {

namespace kf {

CameraIntrinsics loadCameraIntrinsics(const char *fileName) {
    CameraIntrinsics camIntrinsics;
    std::ifstream is(fileName);

    if (is) {
        is >> camIntrinsics.width >> camIntrinsics.height
           >> camIntrinsics.fx >> camIntrinsics.fy
           >> camIntrinsics.cx >> camIntrinsics.cy;
    } else {
        std::cout << "Error loading camera intrinsics data";
    }
    is.close();

    return camIntrinsics;
}

cv::Mat processDepthMap(const cv::Mat& preDepthMap) {
    cv::Mat depthMap(preDepthMap.size(), CV_32F);
    
    float depth;
    for (size_t y = 0; y < depthMap.rows; ++y) {
        for (size_t x = 0; x < depthMap.cols; ++x) {
            depth = preDepthMap.at<ushort>(y, x);
            if (depth == 0) {
                depthMap.at<float>(y, x) = M_INFINITY;
            } else {
                depthMap.at<float>(y, x) = depth * 1.0f / 5000.0f;   
            }
        }
    }

    return depthMap;
}

Camera::Camera(const char *depthFileName, const char *colorFileName,
               const char *camIntrinsicsFileName)
             : frameIndex(-1), depthFileName(depthFileName), colorFileName(colorFileName) {
    camIntrinsics = loadCameraIntrinsics(camIntrinsicsFileName);
}

Frame Camera::getFrame() {
    frameIndex++;

    char fileName[100];

    sprintf(fileName, depthFileName, frameIndex);
    depthMap = cv::imread(fileName, cv::IMREAD_UNCHANGED);

    sprintf(fileName, colorFileName, frameIndex);
    colorMap = cv::imread(fileName, cv::IMREAD_COLOR);

    std::cout << "FRAME #" << frameIndex << std::endl << std::endl;

    depthMap = processDepthMap(depthMap);

    return Frame(depthMap, colorMap, camIntrinsics);
}

} // namespace kf
    
} // namespace cat