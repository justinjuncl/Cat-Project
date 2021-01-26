#ifndef SURFACERECONSTRUCTION_H
#define SURFACERECONSTRUCTION_H

#include <vector>
#include "Util.h"

namespace cat {

namespace kf {

struct Voxel {
    float tsdf;
    int weight;

    Voxel(float tsdf = -1.0f, int weight = 0): tsdf(tsdf), weight(weight) {};
};

struct VolumeParams {
    float        scale = 5.0f / 128.0f;
    cv::Point3i  size = cv::Point3i(128, 128, 128); 
    float        mu = 5.0f / 32.0f;
    int          maxWeight = 128;
};

class Volume {
public:
    Volume(VolumeParams& params, cv::Point3f position) : params(params), position(position) {
        voxelData.resize(params.size.x * params.size.y * params.size.z, Voxel());
    };

    Volume(VolumeParams& params) : Volume(params, cv::Point3f(-params.size.x / 2 * params.scale,
                                                              -params.size.y / 2 * params.scale,
                                                              0.8f)) {
    };

    ~Volume() = default;

    cv::Point3f getVoxelPosition(const size_t x, const size_t y, const size_t z) const {
        return position + cv::Point3f((x + 0.5f) * params.scale,
                                      (y + 0.5f) * params.scale,
                                      (z + 0.5f) * params.scale);
    }

    cv::Point3f getPosition() const {
        return position;
    }

    Voxel& getVoxel(const size_t x, const size_t y, const size_t z) {
        return voxelData[x + y * params.size.x + z * params.size.y * params.size.x];
    }

public:
    const VolumeParams params;

private:
    std::vector<Voxel> voxelData;
    cv::Point3f position;

};

void computeSurfaceReconstruction(const cv::Affine3f& pose, const cv::Mat& depthMap,
                                  const CameraIntrinsics& camIntrinsics,
                                  Volume& volume);
cv::Point2i nearestNeighbourProject(const CameraIntrinsics& camIntrinsics, const cv::Point3f& cameraCoord);
float calculateLambda(const CameraIntrinsics& camIntrinsics, const cv::Point2i& uv);

} // namespace kf
    
} // namespace cat


#endif