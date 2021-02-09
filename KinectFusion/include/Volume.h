#ifndef VOLUME_H
#define VOLUME_H

#include "Util.h"

#include <vector>

namespace cat {

namespace kf {

struct Voxel {
    float tsdf;
    int weight;

    Voxel(float tsdf = 1.0f, int weight = 0): tsdf(tsdf), weight(weight) {};
};

class Volume {
public:
    Volume(VolumeParams& params, cv::Point3f position) : params(params), position(position) {
        voxelData.resize(params.size.x * params.size.y * params.size.z, Voxel());
    };

    Volume(VolumeParams& params) : Volume(params, cv::Point3f(-params.size.x / 2 * params.scale,
                                                              -params.size.y / 2 * params.scale,
                                                              0.0f)) {
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

    Voxel getVoxel(const size_t x, const size_t y, const size_t z) const {
        return voxelData[x + y * params.size.x + z * params.size.y * params.size.x];
    }

public:
    const VolumeParams params;

private:
    std::vector<Voxel> voxelData;
    cv::Point3f position;

};

} // namespace kf
    
} // namespace cat

#endif