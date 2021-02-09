#include "Camera.h"

#include <fstream>
#include <sstream>
#include <iomanip>

namespace cat {

namespace kf {

CameraIntrinsics loadCameraIntrinsics(const std::string fileName) {
    CameraIntrinsics camIntrinsics;
    std::ifstream is(fileName);

    if (is) {
        is >> camIntrinsics.width >> camIntrinsics.height
           >> camIntrinsics.fx >> camIntrinsics.fy
           >> camIntrinsics.cx >> camIntrinsics.cy;
    } else {
        std::cout << "Error loading camera intrinsics data";
    }
    is.close();

    return camIntrinsics;
}

cv::Mat processDepthMap(const cv::Mat& preDepthMap) {
    cv::Mat depthMap(preDepthMap.size(), CV_32F);
    
    float depth;
    for (size_t y = 0; y < depthMap.rows; ++y) {
        for (size_t x = 0; x < depthMap.cols; ++x) {
            depth = preDepthMap.at<ushort>(y, x);
            if (depth == 0) {
                depthMap.at<float>(y, x) = M_INFINITY;
            } else {
                depthMap.at<float>(y, x) = depth * 1.0f / 5000.0f;   
            }
        }
    }

    return depthMap;
}

Camera::Camera(const std::string datasetDirectory,
               const std::string camIntrinsicsFileName)
             : frameIndex(0), datasetDirectory(datasetDirectory) {
    camIntrinsics = loadCameraIntrinsics(camIntrinsicsFileName);
    setupFrames();
}

Frame Camera::getFrame() {
    std::cout << "FRAME #" << frameIndex << " @" << std::fixed << std::setw( 11 ) << std::setprecision( 6 ) << timestampsDepth[frameIndex] << std::endl << std::endl;

    depthMap = cv::imread(fileNamesDepth[frameIndex].c_str(), cv::IMREAD_UNCHANGED);
    colorMap = cv::imread(fileNamesColor[frameIndex].c_str(), cv::IMREAD_COLOR);

    depthMap = processDepthMap(depthMap);

    frameIndex++;

    return Frame(depthMap, colorMap, camIntrinsics);
}

bool Camera::canGetFrame() {
    return frameIndex < fileNamesDepth.size();
}

void Camera::setupFrames() {
    // cd datasetDirectory && python associate.py depth.txt rgb.txt > associate.txt
    std::ifstream associateFile(datasetDirectory + "/associate.txt");

    double timestampDepth, timestampColor;
    std::string fileNameDepth, fileNameColor;

    for (std::string line; std::getline(associateFile, line);) {
        std::stringstream ss(line);
        ss >> timestampDepth >> fileNameDepth >> timestampColor >> fileNameColor;

        timestampsDepth.push_back(timestampDepth);
        fileNamesDepth.push_back(datasetDirectory + "/" + fileNameDepth);
        timestampsColor.push_back(timestampColor);
        fileNamesColor.push_back(datasetDirectory + "/" + fileNameColor);
    }

    associateFile.close();
}

} // namespace kf
    
} // namespace cat