#ifndef SURFACEMEASUREMENT_H
#define SURFACEMEASUREMENT_H

#include "Util.h"
#include "Frame.h"

namespace cat {

namespace kf {

void computeSurfaceMeasurement(Frame& frame, const BilateralFilterParams& filterParams);

} // namespace kf

} // namespace cat

#endif