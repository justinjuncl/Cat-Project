#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "Util.h"
#include "Frame.h"
#include "SurfaceReconstruction.h"

namespace cat {

namespace kf {

class Visualizer {
public:
    Visualizer(const Frame& frame, const Volume& volume);

    void visualizeDepthMap();
    void visualizeNormalMap();
    void visualizeVertexCloud();
    void visualizeVolume();

private:
    cv::viz::Viz3d vizWindow;
    cv::viz::WCloud vertexCloud;

    Frame frame;
    Volume volume;
};

} // namespace kf

} // namespace cat

#endif