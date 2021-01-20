#ifndef SURFACEMEASUREMENT_H
#define SURFACEMEASUREMENT_H

namespace cat {

namespace kf {

struct SurfaceData {
    cv::Mat vertexMap;
    cv::Mat filteredDepthMap;
    cv::Mat normalMap;
};

struct BilateralFilterParams {
    int     d = 7;
    double  sigmaColor = 4.0;
    double  sigmaSpace = 0.25;
};

SurfaceData computeSurfaceMeasurement(const cv::Mat& depthMap, const CameraIntrinsics& camIntrinsics,
                                      const BilateralFilterParams& filterParams);
cv::Mat processDepthMap(const cv::Mat& preDepthMap);
cv::Mat createVertexMap(const cv::Mat& depthMap, const CameraIntrinsics& camIntrinsics);
cv::Mat createNormalMap(const cv::Mat& vertexMap);

} // namespace kf

} // namespace cat

#endif