#include "Util.h"

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

} // namespace kf

} // namespace cat
