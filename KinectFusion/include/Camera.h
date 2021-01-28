#ifndef CAMERA_H
#define CAMERA_H

#include "Util.h"
#include "Frame.h"

namespace cat {

namespace kf {

class Camera {
public:
    Camera(const char *depthFileName, const char *colorFileName,
           const char *camIntrinsicsFileName);
    
    Frame getFrame();

private:
    int frameIndex;
    const char *depthFileName;
    const char *colorFileName;

    CameraIntrinsics camIntrinsics;
    cv::Mat depthMap;
    cv::Mat colorMap;
};

} // namespace kf
    
} // namespace cat

#endif