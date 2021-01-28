#include "Frame.h"

namespace cat {

namespace kf {

Frame::Frame(const cv::Mat& depthMap, const cv::Mat& colorMap, CameraIntrinsics& camIntrinsics)
    : depthMap(depthMap), colorMap(colorMap), camIntrinsics(camIntrinsics) {
    pose = cv::Affine3f().Identity();
}

} // namespace kf
    
} // namespace cat