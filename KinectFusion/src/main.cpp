#include "Util.h"

#include "Camera.h"
#include "Frame.h"
#include "Volume.h"

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

    std::cout << "Surface Reconstruction" << std::endl;
    cat::kf::computeSurfaceReconstruction(currFrame, volume);

    std::cout << "Surface Prediction" << std::endl;
    prevFrame = cat::kf::computeSurfacePrediction(currFrame, volume);

    std::cout << "Pose Estimation" << std::endl << std::endl;
    cat::kf::computePoseEstimation(currFrame, prevFrame, icpParams);
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

        i++;
    }

    cat::kf::Visualizer viz(prevFrame, volume);
    // cv::imshow("Depth 2D Viz", frame.depthMap);
    // cv::imshow("Color", frame.colorMap);
    // viz.visualizeDepthMap();
    // viz.visualizeNormalMap();
    // viz.visualizeVertexCloud();
    viz.showTrajectory(path);
    viz.visualizeVolume();

    return 0;
}