#include "Exporter.h"
#include "MCLookupTable.h"

#include <fstream>

namespace cat {

namespace kf {

struct Grid {
    cv::Point3f p;
    float v;
};

struct Triangle {
    cv::Point3f p1;
    cv::Point3f p2;
    cv::Point3f p3;
};

cv::Point3f vertexInterp(const cv::Point3f& p1, const cv::Point3f& p2, float v1, float v2) {
    return (p1 + (-v1 / (v2 - v1 + 1e-15f)) * (p2 - p1));
}

void Exporter::exportMCMesh(const Volume& volume, const std::string fileName) {
    std::vector<Triangle> faces;
    Grid grid[8];
    cv::Point3f vertexList[12];
    int cubeIndex;

    for (size_t z = 0; z < volume.params.size.z - 1; ++z) {
        for (size_t y = 0; y < volume.params.size.y - 1; ++y) {
            for (size_t x = 0; x < volume.params.size.x - 1; ++x) {
                grid[0].p = volume.getVoxelPosition(x,   y,   z  ); grid[0].v = volume.getVoxel(x,   y,   z  ).tsdf;
                grid[1].p = volume.getVoxelPosition(x,   y,   z+1); grid[1].v = volume.getVoxel(x,   y,   z+1).tsdf;
                grid[2].p = volume.getVoxelPosition(x,   y+1, z  ); grid[2].v = volume.getVoxel(x,   y+1, z  ).tsdf;
                grid[3].p = volume.getVoxelPosition(x,   y+1, z+1); grid[3].v = volume.getVoxel(x,   y+1, z+1).tsdf;
                grid[4].p = volume.getVoxelPosition(x+1, y,   z  ); grid[4].v = volume.getVoxel(x+1, y,   z  ).tsdf;
                grid[5].p = volume.getVoxelPosition(x+1, y,   z+1); grid[5].v = volume.getVoxel(x+1, y,   z+1).tsdf;
                grid[6].p = volume.getVoxelPosition(x+1, y+1, z  ); grid[6].v = volume.getVoxel(x+1, y+1, z  ).tsdf;
                grid[7].p = volume.getVoxelPosition(x+1, y+1, z+1); grid[7].v = volume.getVoxel(x+1, y+1, z+1).tsdf;

                cubeIndex = 0;
                if (grid[0].v < 0.0f) cubeIndex |= 1;
                if (grid[1].v < 0.0f) cubeIndex |= 2;
                if (grid[2].v < 0.0f) cubeIndex |= 4;
                if (grid[3].v < 0.0f) cubeIndex |= 8;
                if (grid[4].v < 0.0f) cubeIndex |= 16;
                if (grid[5].v < 0.0f) cubeIndex |= 32;
                if (grid[6].v < 0.0f) cubeIndex |= 64;
                if (grid[7].v < 0.0f) cubeIndex |= 128;

                if (edgeTable[cubeIndex] == 0) continue;

                if (edgeTable[cubeIndex] & 1)
                    vertexList[0] = vertexInterp(grid[0].p, grid[1].p, grid[0].v, grid[1].v);
                if (edgeTable[cubeIndex] & 2)
                    vertexList[1] = vertexInterp(grid[1].p, grid[2].p, grid[1].v, grid[2].v);
                if (edgeTable[cubeIndex] & 4)
                    vertexList[2] = vertexInterp(grid[2].p, grid[3].p, grid[2].v, grid[3].v);
                if (edgeTable[cubeIndex] & 8)
                    vertexList[3] = vertexInterp(grid[3].p, grid[0].p, grid[3].v, grid[0].v);
                if (edgeTable[cubeIndex] & 16)
                    vertexList[4] = vertexInterp(grid[4].p, grid[5].p, grid[4].v, grid[5].v);
                if (edgeTable[cubeIndex] & 32)
                    vertexList[5] = vertexInterp(grid[5].p, grid[6].p, grid[5].v, grid[6].v);
                if (edgeTable[cubeIndex] & 64)
                    vertexList[6] = vertexInterp(grid[6].p, grid[7].p, grid[6].v, grid[7].v);
                if (edgeTable[cubeIndex] & 128)
                    vertexList[7] = vertexInterp(grid[7].p, grid[4].p, grid[7].v, grid[4].v);
                if (edgeTable[cubeIndex] & 256)
                    vertexList[8] = vertexInterp(grid[0].p, grid[4].p, grid[0].v, grid[4].v);
                if (edgeTable[cubeIndex] & 512)
                    vertexList[9] = vertexInterp(grid[1].p, grid[5].p, grid[1].v, grid[5].v);
                if (edgeTable[cubeIndex] & 1024)
                    vertexList[10] = vertexInterp(grid[2].p, grid[6].p, grid[2].v, grid[6].v);
                if (edgeTable[cubeIndex] & 2048)
                    vertexList[11] = vertexInterp(grid[3].p, grid[7].p, grid[3].v, grid[7].v); 

                for (size_t i = 0; triTable[cubeIndex][i] != -1; i += 3) {
                    faces.push_back({
                        vertexList[triTable[cubeIndex][i  ]],
                        vertexList[triTable[cubeIndex][i+1]],
                        vertexList[triTable[cubeIndex][i+2]]
                    });
                }
            }
        }
    }

    std::ofstream os(fileName);
    if (!os.is_open()) {
        std::cout << "Error loading file " << fileName;
        return;
    }

    for (size_t i = 0; i < faces.size(); ++i) {
        Triangle triangle = faces[i];
        os << "v " << triangle.p1.x << " " << -triangle.p1.y << " " << -triangle.p1.z << std::endl;
        os << "v " << triangle.p2.x << " " << -triangle.p2.y << " " << -triangle.p2.z << std::endl;
        os << "v " << triangle.p3.x << " " << -triangle.p3.y << " " << -triangle.p3.z << std::endl;
    }

    for (size_t i = 0; i < faces.size() * 3; i += 3) {
        os << "f " << i + 1 << " " << i + 2 << " " << i + 3 << std::endl;
    }

    os.close();
}

} // namespace kf

} // namespace cat