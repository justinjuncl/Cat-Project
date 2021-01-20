#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "Util.h"
#include "SurfaceMeasurement.h"
#include "SurfaceReconstruction.h"

namespace cat {

namespace kf {

class Visualizer {
public:
    Visualizer(const SurfaceData& data, const cv::Mat& colorMap,
               const Volume& volume);

    void visualizeDepthMap();
    void visualizeNormalMap();
    void visualizeVertexCloud();
    void visualizeVolume();

private:
    cv::viz::Viz3d vizWindow;
    cv::viz::WCloud vertexCloud;

    SurfaceData data;
    cv::Mat colorMap;
    Volume volume;
};

} // namespace kf

} // namespace cat

#endif