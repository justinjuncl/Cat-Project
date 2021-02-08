#include "PoseEstimation.h"

namespace cat {

namespace kf {

bool solveICP(const Frame& currFrame, const Frame& prevFrame, const ICPParams& icpParams, cv::Mat &x) {
    cv::Mat SigmaA_T_A = cv::Mat::zeros(6, 6, CV_64F);
    cv::Mat SigmaA_T_b = cv::Mat::zeros(6, 1, CV_64F);

    cv::Affine3d currFramePose = currFrame.pose;
    cv::Affine3d prevFramePose = prevFrame.pose;
    cv::Affine3d prevFramePoseInv = prevFramePose.inv();

    int count = 0;

    for (size_t y = 0; y < currFrame.vertexMap.rows; ++y) {
        for (size_t x = 0; x < currFrame.vertexMap.cols; ++x) {
            cv::Vec3d currVertex = currFrame.vertexMap.at<cv::Vec3f>(y, x); // Convert to double
            cv::Vec3d currNormal = currFrame.normalMap.at<cv::Vec3f>(y, x);

            if (isnan(currVertex[0]) || isnan(currNormal[0])) continue;

            cv::Vec3d currVertexGlobal = currFramePose * currVertex;
            cv::Vec3d currNormalGlobal = currFramePose.rotation() * currNormal;

            cv::Point3d currVertexPrev = prevFramePoseInv * currVertexGlobal;

            cv::Point2i uHat = cv::Point2i(round(currVertexPrev.x / currVertexPrev.z
                                               * prevFrame.camIntrinsics.fx + prevFrame.camIntrinsics.cx + 0.5),
                                           round(currVertexPrev.y / currVertexPrev.z
                                               * prevFrame.camIntrinsics.fy + prevFrame.camIntrinsics.cy + 0.5));

            if (uHat.x < 0 || uHat.x >= prevFrame.camIntrinsics.width ||
                uHat.y < 0 || uHat.y >= prevFrame.camIntrinsics.height) continue;

            cv::Vec3d prevVertex = prevFrame.vertexMap.at<cv::Vec3f>(uHat.y, uHat.x);
            cv::Vec3d prevNormal = prevFrame.normalMap.at<cv::Vec3f>(uHat.y, uHat.x);

            if (isnan(prevVertex[0]) || isnan(prevNormal[0])) continue;

            cv::Vec3d prevVertexGlobal = prevVertex; // prevFramePose * prevVertex;
            cv::Vec3d prevNormalGlobal = prevNormal; // prevFramePose.rotation() * prevNormal;

            if (!(cv::norm(currVertexGlobal - prevVertexGlobal) <= icpParams.distanceThreshold &&
                  currNormalGlobal.dot(prevNormalGlobal) >= icpParams.angleThreshold)) continue;

            cv::Mat A_T;
            cv::vconcat(-currVertexGlobal.cross(prevNormalGlobal), prevNormalGlobal, A_T);

            double b = prevNormalGlobal.dot(prevVertexGlobal - currVertexGlobal);

            SigmaA_T_A += A_T * A_T.t();
            SigmaA_T_b += A_T * b;

            count++;
        }
    }

    // std::cout << "SigmaA_T_A: " << std::endl
    //           << SigmaA_T_A << std::endl << std::endl;
    // std::cout << "SigmaA_T_b: " << std::endl
    //           << SigmaA_T_b << std::endl << std::endl;

    bool success = cv::Cholesky(SigmaA_T_A.ptr<double>(), SigmaA_T_A.step, SigmaA_T_A.cols,
                                SigmaA_T_b.ptr<double>(), SigmaA_T_b.step, SigmaA_T_b.cols);

    std::cout << "Cholesky: " << (success ? "Success" : "Fail") << " (" << count << "/"
              << currFrame.vertexMap.rows * currFrame.vertexMap.cols << ")" << std::endl;

    SigmaA_T_b.convertTo(x, CV_32F); // Solution is stored in SigmaA_T_b after calling cv::Cholesky

    // std::cout << x << std::endl;

    return success;
}

cv::Affine3f poseFromSolution(float *x) {
    float alpha = x[0];
    float beta  = x[1];
    float gamma = x[2];

    // float data[16] = { 1.0f,     alpha * beta - gamma, alpha * gamma + beta, x[3],
    //                   gamma, alpha * beta * gamma + 1, beta * gamma - alpha, x[4],
    //                   -beta,                    alpha,                    1, x[5],
    //                       0,                        0,                    0,    1};

    float data[16] = { 1.0f,  x[2], -x[1], x[3],
                      -x[2],  1.0f,  x[0], x[4],
                       x[1], -x[0],  1.0f, x[5],
                       0.0f,  0.0f,  0.0f, 1.0f};
    return cv::Affine3f(data);
}

void computePoseEstimation(Frame& currFrame, const Frame& prevFrame, const ICPParams& icpParams) {
    currFrame.pose = prevFrame.pose;
    cv::Mat x;

    for (size_t i = 0; i < icpParams.iterations; ++i) {
        if (!solveICP(currFrame, prevFrame, icpParams, x)) break;

        cv::Affine3f incPose = poseFromSolution(x.ptr<float>());

        currFrame.pose = incPose * currFrame.pose;
    }
}

} // namespace kf

} // namespace cat