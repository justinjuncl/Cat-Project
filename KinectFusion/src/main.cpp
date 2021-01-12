#include "Util.h"
#include "SurfaceMeasurement.h"
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

    cat::KinectFusion::CameraIntrinsics camIntrinsics = cat::KinectFusion::loadCameraIntrinsics("res/intrinsics.txt");
    cat::KinectFusion::BilateralFilterParams filterParams;

    cat::KinectFusion::SurfaceData data = cat::KinectFusion::computeSurfaceMeasurement(depthMap, camIntrinsics, filterParams);

    cat::KinectFusion::Visualizer viz(data, colorMap);

    // cv::imshow("Depth", depthMap);
    // cv::imshow("Color", colorMap);
    // viz.visualizeDepthMap();
    viz.visualizeNormalMap();
    viz.visualizeVertexCloud();

    return 0;
}
