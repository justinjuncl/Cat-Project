#include "Visualizer.h"

namespace cat {

namespace kf {

Visualizer::Visualizer(const Frame& frame, const Volume& volume) :
    vizWindow(cv::viz::Viz3d("KinectFusion Visualizer")),
    vertexCloud(cv::viz::WCloud(frame.vertexMap, frame.colorMap)),
    frame(frame), volume(volume) {
}

void Visualizer::visualizeDepthMap() {
    cv::Mat normalizedDepthMap;
    frame.filteredDepthMap.convertTo(normalizedDepthMap, CV_16UC3, 5000);
    cv::imshow("Bilateral Filtered Depth 2D Viz", normalizedDepthMap);
}

void Visualizer::visualizeNormalMap() {
    cv::Mat vizNormalMap;
    vizNormalMap = (frame.normalMap + cv::Scalar::all(1.0f)) * 0.5f;
    vizNormalMap.convertTo(vizNormalMap, CV_8UC3, 255);
    cv::cvtColor(vizNormalMap, vizNormalMap, cv::COLOR_BGR2RGB);
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
                float tsdf = volume.getVoxel(x, y, z).tsdf;
                if (-1.0f < tsdf && tsdf < 1.0f) {
                    float t = (tsdf + 1.0f) *  0.5f;
                    t = std::min(std::max(t, 0.0f), 1.0f);
                    cv::Vec3b color = t * green + (1 - t) * red;

                    cv::Point3f voxel = volume.getVoxelPosition(x, y, z);
                    // voxel.y *= -1.0f;
                    // voxel.z *= -1.0f;

                    volumeCloudData.push_back(voxel);
                    volumeCloudColorData.push_back(color);
                }
            }
        }
    }
    cv::viz::WCloud volumeCloud(volumeCloudData, volumeCloudColorData);
    vizWindow.showWidget("Volume Cloud", volumeCloud);
    vizWindow.spin();
}

void Visualizer::visualizeMesh(const std::string fileName) {
    vizWindow.showWidget("Mesh", cv::viz::WMesh(cv::viz::Mesh::load(fileName, cv::viz::Mesh::LOAD_OBJ)));
    vizWindow.spin();;
}

void Visualizer::showTrajectory(const std::vector<cv::Affine3f>& path) {
    cv::Matx33d K = cv::Matx33d(frame.camIntrinsics.fx, 0.0, frame.camIntrinsics.cx,
                                0.0, frame.camIntrinsics.fy, frame.camIntrinsics.cy,
                                0.0,                    0.0,                    1.0); 

    vizWindow.showWidget("Trajectory", cv::viz::WTrajectory(path, cv::viz::WTrajectory::PATH, 0.2, cv::viz::Color::orange()));
    vizWindow.showWidget("Frustrums", cv::viz::WTrajectoryFrustums(path, K, 0.2));
}

} // namespace kf

} // namespace cat