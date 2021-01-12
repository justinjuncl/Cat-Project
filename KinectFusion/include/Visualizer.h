#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "Util.h"
#include "SurfaceMeasurement.h"

namespace cat {

namespace KinectFusion {

class Visualizer {
public:
    Visualizer(const SurfaceData& data, const cv::Mat& colorMap);

    void visualizeDepthMap();
    void visualizeNormalMap();
    void visualizeVertexCloud();

private:
    cv::viz::Viz3d vizWindow;
    cv::viz::WCloud vertexCloud;
    cv::viz::WCloudNormals normalCloud;

    SurfaceData data;
    cv::Mat colorMap;
};

} // namespace KinectFusion

} // namespace cat

#endif