#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/viz.hpp>

#ifndef M_INFINITY
#define M_INFINITY -std::numeric_limits<float>::infinity()
#endif

namespace cat {

namespace kf {

struct CameraIntrinsics {
    int width, height;
    float fx, fy, cx, cy;
};

struct BilateralFilterParams {
    int     d = 7;
    double  sigmaColor = 4.0;
    double  sigmaSpace = 0.25;
};

struct VolumeParams {
    float        scale = 5.0f / 256.0f;
    cv::Point3i  size = cv::Point3i(256, 256, 256); 
    float        mu = 5.0f / 64.0f;
    int          maxWeight = 128;
};

struct ICPParams {
    size_t iterations = 10;
    float distanceThreshold = 0.1f;
    float angleThreshold = 0.8f;
};

} // namespace kf

} // namespace cat

#endif