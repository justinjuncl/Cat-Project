#include "Util.h"
#include "SurfaceMeasurement.h"

namespace cat {

namespace kf {

SurfaceData computeSurfaceMeasurement(const cv::Mat& preDepthMap, const CameraIntrinsics& camIntrinsics,
                                      const BilateralFilterParams& filterParams) {
    cv::Mat depthMap = processDepthMap(preDepthMap);

    cv::Mat filteredDepthMap;
    cv::bilateralFilter(depthMap, filteredDepthMap, filterParams.d, filterParams.sigmaColor, filterParams.sigmaSpace);

    cv::Mat vertexMap = createVertexMap(filteredDepthMap, camIntrinsics);
    cv::Mat normalMap = createNormalMap(vertexMap);

    SurfaceData data;
    data.vertexMap = vertexMap;
    data.filteredDepthMap = filteredDepthMap;
    data.normalMap = normalMap;

    return data;
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

cv::Mat createVertexMap(const cv::Mat& depthMap, const CameraIntrinsics& camIntrinsics) {
    cv::Mat vertexMap(depthMap.size(), CV_32FC3);

    float depth;
    for (size_t y = 0; y < vertexMap.rows; ++y) {
        for (size_t x = 0; x < vertexMap.cols; ++x) {
            depth = depthMap.at<float>(y, x);
            vertexMap.at<cv::Vec3f>(y, x) = cv::Vec3f(depth * (x - camIntrinsics.cx) / camIntrinsics.fx,
                                                      depth * (y - camIntrinsics.cy) / camIntrinsics.fy,
                                                      depth);
        }
    }

    return vertexMap;
}

cv::Mat createNormalMap(const cv::Mat& vertexMap) {
    cv::Mat normalMap(vertexMap.size(), CV_32FC3);

    for (size_t y = 0; y < normalMap.rows; ++y) {
        for (size_t x = 0; x < normalMap.cols; ++x) {
            if (x == 0 || y == 0 || x == normalMap.cols - 1 || y == normalMap.rows - 1) {
                normalMap.at<cv::Vec3f>(y, x) = cv::Vec3f(M_INFINITY, M_INFINITY, M_INFINITY);
            } else {
                auto du = vertexMap.at<cv::Vec3f>(y, x+1) - vertexMap.at<cv::Vec3f>(y, x-1);
                auto dv = vertexMap.at<cv::Vec3f>(y+1, x) - vertexMap.at<cv::Vec3f>(y-1, x);
                auto n = du.cross(dv);
                cv::normalize(n, n);

                if (isnan(n[0]) || isnan(n[1]) || isnan(n[2])) {
                    normalMap.at<cv::Vec3f>(y, x) = cv::Vec3f(M_INFINITY, M_INFINITY, M_INFINITY);
                } else {
                    normalMap.at<cv::Vec3f>(y, x) = n;
                }
            }
        }
    }
    return normalMap;
}

} // namespace kf

} // namespace cat