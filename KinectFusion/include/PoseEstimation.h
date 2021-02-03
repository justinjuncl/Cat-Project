#ifndef POSEESTIMATION_H
#define POSEESTIMATION_H

#include "Util.h"
#include "Frame.h"

namespace cat {

namespace kf {

void computePoseEstimation(Frame& currFrame, const Frame& prevFrame, const ICPParams& icpParams);

} // namespace kf

} // namespace cat

#endif