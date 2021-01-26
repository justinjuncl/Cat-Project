#include "Util.h"
#include "SurfaceMeasurement.h"
#include "SurfaceReconstruction.h"
#include "SurfacePrediction.h"
#include "Visualizer.h"

int main(int, char**) {
    cv::Mat depthMap = cv::imread("res/images/depth-0.png", cv::IMREAD_UNCHANGED);
    cv::Mat colorMap = cv::imread("res/images/color-0.png", cv::IMREAD_COLOR);

    std::cout << "depthMap: " << std::endl
                   << "chn: " << depthMap.channels() << std::endl
                   << "typ: " << depthMap.type() << std::endl << std::endl;

    std::cout << "colorMap: " << std::endl
                   << "chn: " << colorMap.channels() << std::endl
                   << "typ: " << colorMap.type() << std::endl << std::endl;

    cat::kf::CameraIntrinsics camIntrinsics = cat::kf::loadCameraIntrinsics("res/intrinsics.txt");
    cat::kf::BilateralFilterParams filterParams;

    cat::kf::SurfaceData data = cat::kf::computeSurfaceMeasurement(depthMap, camIntrinsics, filterParams);

    cat::kf::VolumeParams volumeParams;
    cat::kf::Volume volume(volumeParams);

    cv::Affine3f pose = cv::Affine3f().Identity();

    std::cout << "pose:" << std::endl
              << pose.rotation() << std::endl
              << pose.translation() << std::endl << std::endl;

    cat::kf::computeSurfaceReconstruction(pose, cat::kf::processDepthMap(depthMap), camIntrinsics, volume);
    cat::kf::SurfaceData data_pred = cat::kf::computeSurfacePrediction(pose, camIntrinsics, volume);

    cat::kf::Visualizer viz(data_pred, colorMap, volume);


    // cv::imshow("Depth 2D Viz", depthMap);
    // cv::imshow("Color", colorMap);
    // viz.visualizeDepthMap();
    viz.visualizeNormalMap();
    viz.visualizeVertexCloud();
    // viz.visualizeVolume();

    return 0;
}
