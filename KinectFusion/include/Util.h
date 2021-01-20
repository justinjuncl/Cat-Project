#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <fstream>
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

CameraIntrinsics loadCameraIntrinsics(const char *fileName);

} // namespace kf

} // namespace cat

#endif