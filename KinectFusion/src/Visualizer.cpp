#include "Visualizer.h"

namespace cat {

namespace kf {

Visualizer::Visualizer(const SurfaceData& data, const cv::Mat& colorMap,
                       const Volume& volume) :
    vizWindow(cv::viz::Viz3d("Vertex Viz")),
    vertexCloud(cv::viz::WCloud(data.vertexMap, colorMap)),
    data(data), colorMap(colorMap), volume(volume) {
}

void Visualizer::visualizeDepthMap() {
    cv::Mat normalizedDepthMap;
    data.filteredDepthMap.convertTo(normalizedDepthMap, CV_16UC3, 5000);
    cv::imshow("Bilateral Filtered Depth 2D Viz", normalizedDepthMap);
}

void Visualizer::visualizeNormalMap() {
    cv::Mat vizNormalMap;
    data.normalMap = (data.normalMap + cv::Scalar::all(1.0f)) * 0.5f;
    data.normalMap.convertTo(vizNormalMap, CV_8UC3, 255);
    cv::imshow("Normal 2D Viz", vizNormalMap);
}

void Visualizer::visualizeVertexCloud() {
    vizWindow.showWidget("Vertex Cloud", vertexCloud);
    vizWindow.spin();
}

void Visualizer::visualizeVolume() {
    std::vector<cv::Point3f> volumeCloudData;
    std::vector<cv::Vec3b> volumeCloudColorData;
    cv::Vec3b red(0, 0, 255);
    cv::Vec3b green(0, 255, 0);

    for (size_t z = 0; z < volume.params.size.z; ++z) {
        for (size_t y = 0; y < volume.params.size.y; ++y) {
            for (size_t x = 0; x < volume.params.size.x; ++x) {
                if (volume.getVoxel(x, y, z).tsdf < 0) {
                    float t = -volume.getVoxel(x, y, z).tsdf / volume.params.mu;
                    t = std::min(std::max(t, 0.0f), 1.0f);
                    cv::Vec3b color = t * green + (1 - t) * red;

                    volumeCloudData.push_back(volume.getVoxelPosition(x, y, z));
                    volumeCloudColorData.push_back(color);
                }
            }
        }
    }
    cv::viz::WCloud volumeCloud(volumeCloudData, volumeCloudColorData);
    vizWindow.showWidget("Volume Cloud", volumeCloud);
    vizWindow.spin();
}

} // namespace kf

} // namespace cat