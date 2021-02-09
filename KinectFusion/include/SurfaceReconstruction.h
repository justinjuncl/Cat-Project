#ifndef SURFACERECONSTRUCTION_H
#define SURFACERECONSTRUCTION_H

#include "Util.h"
#include "Frame.h"
#include "Volume.h"

namespace cat {

namespace kf {

void computeSurfaceReconstruction(const Frame& frame, Volume& volume);

} // namespace kf
    
} // namespace cat

#endif