#include "Util.h"

#include "Frame.h"
#include "Camera.h"

#include "SurfaceMeasurement.h"
#include "PoseEstimation.h"
#include "SurfaceReconstruction.h"
#include "SurfacePrediction.h"

#include "Visualizer.h"

cat::kf::BilateralFilterParams filterParams;
cat::kf::VolumeParams volumeParams;
cat::kf::ICPParams icpParams;

void pipeline(cat::kf::Frame& currFrame, cat::kf::Frame& prevFrame, cat::kf::Volume& volume) {
    std::cout << "Surface Measurement" << std::endl;
    cat::kf::computeSurfaceMeasurement(currFrame, filterParams);

    std::cout << "Pose Estimation" << std::endl;
    cat::kf::computePoseEstimation(currFrame, prevFrame, icpParams);

    std::cout << "Surface Reconstruction" << std::endl;
    cat::kf::computeSurfaceReconstruction(currFrame, volume);

    std::cout << "Surface Prediction" << std::endl << std::endl;
    prevFrame = cat::kf::computeSurfacePrediction(currFrame, volume);
}

int main(int, char**) {
    cat::kf::Camera camera("/Users/justinjun/Downloads/rgbd_dataset_freiburg1_xyz", "res/intrinsics.txt");

    cat::kf::Volume volume(volumeParams);

    cat::kf::Frame prevFrame = camera.getFrame();

    std::cout << "Surface Measurement" << std::endl;
    cat::kf::computeSurfaceMeasurement(prevFrame, filterParams);

    std::vector<cv::Affine3f> path;

    int i = 0;
    while (camera.canGetFrame() && i < 5) {
        cat::kf::Frame currFrame = camera.getFrame();

        pipeline(currFrame, prevFrame, volume);

        std::cout << "currFrame.pose" << std::endl
                  << currFrame.pose.rotation() << std::endl
                  << currFrame.pose.translation() << std::endl << std::endl;

        path.push_back(currFrame.pose);

        cv::Mat vizNormalMap;
        vizNormalMap = (prevFrame.normalMap + cv::Scalar::all(1.0f)) * 0.5f;
        vizNormalMap.convertTo(vizNormalMap, CV_8UC3, 255);
        cv::cvtColor(vizNormalMap, vizNormalMap, cv::COLOR_BGR2RGB);
        cv::imshow(std::to_string(++i) + " Normal 2D", vizNormalMap);
    }

    cv::viz::Viz3d vizWindow("Path Viz");

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
    
    cv::Matx33d K = cv::Matx33d(prevFrame.camIntrinsics.fx, 0, prevFrame.camIntrinsics.cx,
                                0, prevFrame.camIntrinsics.fy, prevFrame.camIntrinsics.cy,
                                0,                          0,                          1);
    cv::viz::WTrajectoryFrustums frustrums(path, K, 0.2);

    vizWindow.showWidget("Trajectory", cv::viz::WTrajectory(path, cv::viz::WTrajectory::PATH, 0.2, cv::viz::Color::orange()));
    vizWindow.showWidget("Frustrums", frustrums);
    vizWindow.showWidget("Volume Cloud", volumeCloud);
    vizWindow.spin(); 

    // cat::kf::Visualizer viz(prevFrame, volume);
    // cv::imshow("Depth 2D Viz", frame.depthMap);
    // cv::imshow("Color", frame.colorMap);
    // viz.visualizeDepthMap();
    // viz.visualizeNormalMap();
    // viz.visualizeVertexCloud();
    // viz.visualizeVolume();

    return 0;
}