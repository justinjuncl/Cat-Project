#include "Visualizer.h"

namespace cat {

namespace KinectFusion {

Visualizer::Visualizer(const SurfaceData& data, const cv::Mat& colorMap) :
    vizWindow(cv::viz::Viz3d("Vertex Viz")),
    vertexCloud(cv::viz::WCloud(data.vertexMap, colorMap)),
    normalCloud(cv::viz::WCloudNormals(data.vertexMap, data.normalMap)),
    data(data), colorMap(colorMap) {
}

void Visualizer::visualizeDepthMap() {
    cv::Mat normalizedDepthMap;
    data.filteredDepthMap.convertTo(normalizedDepthMap, CV_16UC3, 5000);
    cv::imshow("Bilateral Filtered Depth 2D Viz", normalizedDepthMap);
}

void Visualizer::visualizeNormalMap() {
    cv::Mat vizNormalMap;
    data.normalMap.convertTo(vizNormalMap, CV_8UC3, 255);
    cv::imshow("Normal 2D Viz", vizNormalMap);
}

void Visualizer::visualizeVertexCloud() {
    vizWindow.showWidget("Vertex Cloud", vertexCloud);
    // vizWindow.showWidget("Normal Cloud", normalCloud);

    vizWindow.spin();
}

} // namespace KinectFusion

} // namespace cat