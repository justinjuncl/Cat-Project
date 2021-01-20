#include "Util.h"
#include "SurfaceReconstruction.h"

namespace cat {

namespace kf {

cv::Point2i nearestNeighbourProject(const CameraIntrinsics& camIntrinsics, const cv::Point3f& cameraCoord) {
    return cv::Point2i(round(cameraCoord.x / cameraCoord.z * camIntrinsics.fx + camIntrinsics.cx),
                       round(cameraCoord.y / cameraCoord.z * camIntrinsics.fy + camIntrinsics.cy));
}

float calculateLambda(const CameraIntrinsics& camIntrinsics, const cv::Point2i& uv) {
    cv::Point3f lambdaVector = cv::Point3f((uv.x - camIntrinsics.cx) / camIntrinsics.fx,
                                           (uv.y - camIntrinsics.cy) / camIntrinsics.fy,
                                           1.0f);
    return cv::norm(lambdaVector);
}

void computeSurfaceReconstruction(const cv::Affine3f& pose, const cv::Mat& depthMap,
                                  const CameraIntrinsics& camIntrinsics,
                                  Volume& volume) {
    cv::Affine3f poseInv = pose.inv();

    for (size_t z = 0; z < volume.params.size.z; ++z) {
        for (size_t y = 0; y < volume.params.size.y; ++y) {
            for (size_t x = 0; x < volume.params.size.x; ++x) {

                cv::Point3f p = volume.getVoxelPosition(x, y, z);
                cv::Point3f pCameraCoordinate = poseInv * p;

                if (pCameraCoordinate.z <= 0) continue;

                cv::Point2i uv = nearestNeighbourProject(camIntrinsics, pCameraCoordinate);

                if (uv.x < 0 || uv.x >= camIntrinsics.width || uv.y < 0 || uv.y >= camIntrinsics.height) continue;

                float lambda = calculateLambda(camIntrinsics, uv);
                float depth = depthMap.at<float>(uv.y, uv.x);

                if (depth <= 0) continue;

                float eta = (-1.0f) * ((1.0f / lambda) * cv::norm(pCameraCoordinate) - depth); 

                if (eta >= -volume.params.mu) {
                    float currTSDF = std::min(1.0f, eta / volume.params.mu);
                    int currWeight = 1;

                    Voxel oldVoxel = volume.getVoxel(x, y, z);

                    float newTSDF = (oldVoxel.weight * oldVoxel.tsdf + currWeight * currTSDF)
                                    / (oldVoxel.weight + currWeight);
                    int newWeight = std::min(oldVoxel.weight + currWeight, volume.params.maxWeight);

                    volume.getVoxel(x, y, z) = Voxel(newTSDF, newWeight);
                }
            }
        }
    }

    std::cout << "Complete" << std::endl;
}

} // namespace kf

} // namespace cat