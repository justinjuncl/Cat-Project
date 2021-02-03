#include "Util.h"
#include "SurfacePrediction.h"
#include "SurfaceMeasurement.h"

namespace cat {

namespace kf {

float sampleVoxel(const Volume& volume, const cv::Point3f& grid) {
    cv::Point3i voxel = cv::Point3i(static_cast<int>(grid.x + 0.5f) - 1,
                                    static_cast<int>(grid.y + 0.5f) - 1,
                                    static_cast<int>(grid.z + 0.5f) - 1);

    float a = grid.x - (voxel.x + 0.5f);
    float b = grid.y - (voxel.y + 0.5f);
    float c = grid.z - (voxel.z + 0.5f);

    return volume.getVoxel(voxel.x,   voxel.y,   voxel.z  ).tsdf * (1-a)*(1-b)*(1-c)
         + volume.getVoxel(voxel.x,   voxel.y,   voxel.z+1).tsdf * (1-a)*(1-b)*(c)
         + volume.getVoxel(voxel.x,   voxel.y+1, voxel.z  ).tsdf * (1-a)*(b)*(1-c)
         + volume.getVoxel(voxel.x,   voxel.y+1, voxel.z+1).tsdf * (1-a)*(b)*(c)
         + volume.getVoxel(voxel.x+1, voxel.y,   voxel.z  ).tsdf * (a)*(1-b)*(1-c)
         + volume.getVoxel(voxel.x+1, voxel.y,   voxel.z+1).tsdf * (a)*(1-b)*(c)
         + volume.getVoxel(voxel.x+1, voxel.y+1, voxel.z  ).tsdf * (a)*(b)*(1-c)
         + volume.getVoxel(voxel.x+1, voxel.y+1, voxel.z+1).tsdf * (a)*(b)*(c);

    // return ( volume.getVoxel(voxel.x,   voxel.y,   voxel.z  ).tsdf == 1.0f ? 0.0f : volume.getVoxel(voxel.x,   voxel.y,   voxel.z  ).tsdf) * (1-a)*(1-b)*(1-c)
    //      + ( volume.getVoxel(voxel.x,   voxel.y,   voxel.z+1).tsdf == 1.0f ? 0.0f : volume.getVoxel(voxel.x,   voxel.y,   voxel.z+1).tsdf) * (1-a)*(1-b)*(c)
    //      + ( volume.getVoxel(voxel.x,   voxel.y+1, voxel.z  ).tsdf == 1.0f ? 0.0f : volume.getVoxel(voxel.x,   voxel.y+1, voxel.z  ).tsdf) * (1-a)*(b)*(1-c)
    //      + ( volume.getVoxel(voxel.x,   voxel.y+1, voxel.z+1).tsdf == 1.0f ? 0.0f : volume.getVoxel(voxel.x,   voxel.y+1, voxel.z+1).tsdf) * (1-a)*(b)*(c)
    //      + ( volume.getVoxel(voxel.x+1, voxel.y,   voxel.z  ).tsdf == 1.0f ? 0.0f : volume.getVoxel(voxel.x+1, voxel.y,   voxel.z  ).tsdf) * (a)*(1-b)*(1-c)
    //      + ( volume.getVoxel(voxel.x+1, voxel.y,   voxel.z+1).tsdf == 1.0f ? 0.0f : volume.getVoxel(voxel.x+1, voxel.y,   voxel.z+1).tsdf) * (a)*(1-b)*(c)
    //      + ( volume.getVoxel(voxel.x+1, voxel.y+1, voxel.z  ).tsdf == 1.0f ? 0.0f : volume.getVoxel(voxel.x+1, voxel.y+1, voxel.z  ).tsdf) * (a)*(b)*(1-c)
    //      + ( volume.getVoxel(voxel.x+1, voxel.y+1, voxel.z+1).tsdf == 1.0f ? 0.0f : volume.getVoxel(voxel.x+1, voxel.y+1, voxel.z+1).tsdf) * (a)*(b)*(c);
}

bool outOfBoundsRay(const cv::Point3f& minVoxel, const cv::Point3f& maxVoxel,
                    const cv::Vec3f& rayOrigin, const cv::Vec3f& rayDirection, float& t) {
    float invRayDir_x = 1.0f / rayDirection[0];
    float invRayDir_y = 1.0f / rayDirection[1];
    float invRayDir_z = 1.0f / rayDirection[2];

    float t_min_x = ((invRayDir_x > 0.0f ? minVoxel.x : maxVoxel.x) - rayOrigin[0]) * invRayDir_x;
    float t_min_y = ((invRayDir_y > 0.0f ? minVoxel.y : maxVoxel.y) - rayOrigin[1]) * invRayDir_y;
    float t_min_z = ((invRayDir_z > 0.0f ? minVoxel.z : maxVoxel.z) - rayOrigin[2]) * invRayDir_z;

    float t_min = std::fmax(std::fmax(std::fmax(t_min_x, t_min_y), t_min_z), 0.0f);

    float t_max_x = ((invRayDir_x < 0.0f ? minVoxel.x : maxVoxel.x) - rayOrigin[0]) * invRayDir_x;
    float t_max_y = ((invRayDir_y < 0.0f ? minVoxel.y : maxVoxel.y) - rayOrigin[1]) * invRayDir_y;
    float t_max_z = ((invRayDir_z < 0.0f ? minVoxel.z : maxVoxel.z) - rayOrigin[2]) * invRayDir_z;

    float t_max = std::fmin(std::fmin(t_max_x, t_max_y), t_max_z);

    t = t_min;
    return t >= t_max;
}

inline bool outOfBounds(const Volume& volume, const cv::Point3f& pLocal) {
    return (pLocal.x <= 0.0f || pLocal.x >= volume.params.size.x - 1.0f ||
            pLocal.y <= 0.0f || pLocal.y >= volume.params.size.y - 1.0f ||
            pLocal.z <= 0.0f || pLocal.z >= volume.params.size.z - 1.0f);
}

inline bool outOfBoundsNeighbor(const Volume& volume, const cv::Point3f& pLocal) {
    return (pLocal.x <= 1.0f || pLocal.x >= volume.params.size.x - 2.0f ||
            pLocal.y <= 1.0f || pLocal.y >= volume.params.size.y - 2.0f ||
            pLocal.z <= 1.0f || pLocal.z >= volume.params.size.z - 2.0f);
}

cv::Vec3f calculateNormal(const Volume& volume, const cv::Point3f& pLocal) {
    cv::Vec3f n;
    cv::Point3f temp;

    temp = pLocal;
    temp.x += 1.0f;
    float tsdf_x1 = sampleVoxel(volume, temp);
    temp = pLocal;
    temp.x -= 1.0f;
    float tsdf_x2 = sampleVoxel(volume, temp);
    n[0] = tsdf_x2 - tsdf_x1;

    temp = pLocal;
    temp.y += 1.0f;
    float tsdf_y1 = sampleVoxel(volume, temp);
    temp = pLocal;
    temp.y -= 1.0f;
    float tsdf_y2 = sampleVoxel(volume, temp);
    n[1] = tsdf_y2 - tsdf_y1;

    temp = pLocal;
    temp.z += 1.0f;
    float tsdf_z1 = sampleVoxel(volume, temp);
    temp = pLocal;
    temp.z -= 1.0f;
    float tsdf_z2 = sampleVoxel(volume, temp);
    n[2] = tsdf_z2 - tsdf_z1;

    if (cv::norm(n) == 0.0f || isnan(n[0]) || isnan(n[1]) || isnan(n[2]))
        return cv::Vec3f(M_INFINITY, M_INFINITY, M_INFINITY);

    cv::normalize(n, n);

    return n;
}

Frame computeSurfacePrediction(const Frame& frame, const Volume& volume) {
    cv::Mat vertexMap(frame.camIntrinsics.height, frame.camIntrinsics.width, CV_32FC3, cv::Vec3f(M_INFINITY, M_INFINITY, M_INFINITY));
    cv::Mat normalMap(frame.camIntrinsics.height, frame.camIntrinsics.width, CV_32FC3, cv::Vec3f(M_INFINITY, M_INFINITY, M_INFINITY));

    cv::Vec3f volumePosition = cv::Vec3f(volume.getPosition());
    cv::Point3f minVoxelPosition = volume.getVoxelPosition(0, 0, 0);
    cv::Point3f maxVoxelPosition = volume.getVoxelPosition(volume.params.size.x - 1,
                                                           volume.params.size.y - 1,
                                                           volume.params.size.z - 1);

    float t = 0.0f;
    float deltaT = volume.params.mu * 0.5f;
    float maxT = cv::norm(volume.params.size);

    for (size_t y = 0; y < vertexMap.rows; ++y) {
        for (size_t x = 0; x < vertexMap.cols; ++x) {
            cv::Vec3f rayOrigin = frame.pose.translation();
            cv::Vec3f rayDirection = frame.pose.rotation() * cv::Vec3f((x - frame.camIntrinsics.cx) / frame.camIntrinsics.fx,
                                                                       (y - frame.camIntrinsics.cy) / frame.camIntrinsics.fy,
                                                                       1.0f);
            cv::normalize(rayDirection, rayDirection);

            if (outOfBoundsRay(minVoxelPosition, maxVoxelPosition, rayOrigin, rayDirection, t)) continue;

            t += deltaT; // Increment one step for trilinear sampling

            cv::Point3f pLocal = (rayOrigin + (rayDirection * t) - volumePosition) / volume.params.scale;

            float currTSDF = sampleVoxel(volume, pLocal);
            // float currTSDF = volume.getVoxel(round(pLocal.x), round(pLocal.y), round(pLocal.z)).tsdf;

            for (; t < maxT; t += deltaT) {
                pLocal = (rayOrigin + (rayDirection * (t + deltaT)) - volumePosition) / volume.params.scale;

                if (outOfBoundsNeighbor(volume, pLocal)) break;

                float prevTSDF = currTSDF;
                currTSDF = sampleVoxel(volume, pLocal);
                // currTSDF = volume.getVoxel(round(pLocal.x), round(pLocal.y), round(pLocal.z)).tsdf;

                if (prevTSDF < 0.0f && currTSDF > 0.0f) break;
                if (prevTSDF > 0.0f && currTSDF < 0.0f) {
                    float tStar = t - deltaT * prevTSDF / (currTSDF - prevTSDF);
                    cv::Vec3f q = rayOrigin + (rayDirection * tStar);
                    cv::Vec3f qLocal = (q - volumePosition) / volume.params.scale;

                    if (outOfBoundsNeighbor(volume, qLocal)) break;

                    vertexMap.at<cv::Vec3f>(y, x) = q;
                    normalMap.at<cv::Vec3f>(y, x) = calculateNormal(volume, qLocal);

                    break;
                }
            }
        }
    }

    Frame newFrame(frame);

    newFrame.vertexMap = vertexMap;
    newFrame.normalMap = normalMap;

    return newFrame; 
}

} // namespace kf
    
} // namespace cat