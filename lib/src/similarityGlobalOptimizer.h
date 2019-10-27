#ifndef HABITRACK_SIMILARITY_GLOBAL_OPTIMIZER_H
#define HABITRACK_SIMILARITY_GLOBAL_OPTIMIZER_H

#include <Eigen/Dense>

#include "metrics.h"

namespace ht
{
class SimilarityGlobalFunctor
{
public:
    SimilarityGlobalFunctor(const Eigen::Vector2d& q1, const Eigen::Vector2d& q2);

    template <typename T>
    bool operator()(const T* const camParams, const T* const distParams, const T* const simVec0,
        const T* const simVec1, T* errors) const
    {
        (void)camParams;
        (void)distParams;
        using Mat3 = Eigen::Matrix<T, 3, 3, Eigen::RowMajor>;
        using Vec2 = Eigen::Matrix<T, 2, 1>;

        Mat3 homMat0;
        homMat0 << simVec0[0], simVec0[1], simVec0[2], -simVec0[1], simVec0[0], simVec0[5], T(0),
            T(0), T(1);

        Mat3 homMat1;
        homMat1 << simVec1[0], simVec1[1], simVec1[2], -simVec1[1], simVec1[0], simVec1[5], T(0),
            T(0), T(1);

        Vec2 mQ1T(mQ1(0), mQ1(1));
        Vec2 mQ2T(mQ2(0), mQ2(1));
        mQ1T = undistort<T>(camParams, distParams, mQ1T);
        mQ2T = undistort<T>(camParams, distParams, mQ2T);

        Mat3 camMat;
        camMat << camParams[0], T(0), camParams[2], T(0), camParams[1], camParams[3], T(0), T(0),
            T(1);

        Mat3 transformation = camMat * homMat1.inverse() * homMat0 * camMat.inverse();
        symmetricReprojectionError<T>(transformation, mQ1T, mQ2T, errors);
        return true;
    }

private:
    const Eigen::Vector2d mQ1;
    const Eigen::Vector2d mQ2;
};
} // namespace ht
#endif // HABITRACK_SIMILARITY_GLOBAL_OPTIMIZER_H
