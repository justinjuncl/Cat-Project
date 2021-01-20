#ifndef SURFACERECONSTRUCTION_H
#define SURFACERECONSTRUCTION_H

#include <vector>
#include "Util.h"

namespace cat {

namespace kf {

struct Voxel {
    float tsdf;
    int weight;

    Voxel() = default;
    Voxel(float tsdf, int weight): tsdf(tsdf), weight(weight) {};
};

struct VolumeParams {
    float        scale = 5.0f / 128.0f;
    cv::Point3i  size = cv::Point3i(128, 128, 128); 
    float        mu = 0.1f;
    int          maxWeight = 128;
};

class Volume {
public:
    Volume(VolumeParams& params) : params(params) {
        voxelData.resize(params.size.x * params.size.y * params.size.z, Voxel());
    };
    ~Volume() = default;

    cv::Point3f getVoxelPosition(const size_t x, const size_t y, const size_t z) {
        return cv::Point3f((static_cast<float>(x) + 0.5f) * params.scale,
                           (static_cast<float>(y) + 0.5f) * params.scale,
                           (static_cast<float>(z) + 0.5f) * params.scale);
    }

    Voxel& getVoxel(const size_t x, const size_t y, const size_t z) {
        return voxelData[x + y * params.size.x + z * params.size.y * params.size.x];
    }

    std::vector<Voxel>& getVoxelData() {
        return voxelData;
    }

public:
    const VolumeParams params;

private:
    std::vector<Voxel> voxelData;

};

cv::Point2i nearestNeighbourProject(const CameraIntrinsics& camIntrinsics, const cv::Point3f& cameraCoord);
float calculateLambda(const CameraIntrinsics& camIntrinsics, const cv::Point2i& uv);
void computeSurfaceReconstruction(const cv::Affine3f& pose, const cv::Mat& depthMap,
                                  const CameraIntrinsics& camIntrinsics,
                                  Volume& volume);

} // namespace kf
    
} // namespace cat


#endif