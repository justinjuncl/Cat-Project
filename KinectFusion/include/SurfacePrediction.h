#ifndef SURFACEPREDICTION_H
#define SURFACEPREDICTION_H

#include "Util.h"
#include "Frame.h"
#include "SurfaceReconstruction.h"

namespace cat {

namespace kf {

Frame computeSurfacePrediction(const Frame& frame, const Volume& volume);

} // namespace kf
    
} // namespace cat

#endif