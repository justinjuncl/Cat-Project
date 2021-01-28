#include "Util.h"

#include "Frame.h"
#include "Camera.h"

#include "SurfaceMeasurement.h"
#include "SurfaceReconstruction.h"
#include "SurfacePrediction.h"

#include "Visualizer.h"

int main(int, char**) {
    cat::kf::Camera camera("res/images/depth-%d.png", "res/images/color-%d.png", "res/intrinsics.txt");
    cat::kf::Frame frame = camera.getFrame();

    cat::kf::VolumeParams volumeParams;
    cat::kf::Volume volume(volumeParams);

    cat::kf::BilateralFilterParams filterParams;

    cat::kf::computeSurfaceMeasurement(frame, filterParams);
    cat::kf::computeSurfaceReconstruction(frame, volume);
    cat::kf::Frame prevFrame = cat::kf::computeSurfacePrediction(frame, volume);

    cat::kf::Visualizer viz(prevFrame, volume);
    // cv::imshow("Depth 2D Viz", frame.depthMap);
    // cv::imshow("Color", frame.colorMap);
    viz.visualizeDepthMap();
    viz.visualizeNormalMap();
    // viz.visualizeVertexCloud();
    viz.visualizeVolume();

    return 0;
}