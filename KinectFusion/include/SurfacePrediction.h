#ifndef SURFACEPREDICTION_H
#define SURFACEPREDICTION_H

#include "Util.h"
#include "SurfaceMeasurement.h"
#include "SurfaceReconstruction.h"

namespace cat {

namespace kf {


SurfaceData computeSurfacePrediction(const cv::Affine3f& pose,
                                     const CameraIntrinsics& camIntrinsics,
                                     const Volume& volume);

} // namespace kf
    
} // namespace cat


#endif