#ifndef HABITRACK_ISOMETRY_GLOBAL_OPTIMIZER_H
#define HABITRACK_ISOMETRY_GLOBAL_OPTIMIZER_H

#include "metrics.h"
#include <Eigen/Dense>

namespace ht
{
class IsometryGlobalFunctor
{
public:
    IsometryGlobalFunctor(const Eigen::Vector2d& q1, const Eigen::Vector2d& q2, double weight);

    template <typename T>
    bool operator()(const T* const camParams, const T* const distParams, const T* const isoVec0,
        const T* const isoVec1, T* errors) const
    {
        (void)camParams;
        (void)distParams;
        using Mat3 = Eigen::Matrix<T, 3, 3, Eigen::RowMajor>;
        using Vec2 = Eigen::Matrix<T, 2, 1>;

        T angle0 = isoVec0[0];
        T angle1 = isoVec1[0];
        Mat3 homMat0;
        homMat0 << cos(angle0), sin(angle0), isoVec0[1], -sin(angle0), cos(angle0), isoVec0[2],
            T(0), T(0), T(1);

        Mat3 homMat1;
        homMat1 << cos(angle1), sin(angle1), isoVec1[1], -sin(angle1), cos(angle1), isoVec1[2],
            T(0), T(0), T(1);

        Vec2 mQ1T(mQ1(0), mQ1(1));
        Vec2 mQ2T(mQ2(0), mQ2(1));
        mQ1T = undistort<T>(camParams, distParams, mQ1T);
        mQ2T = undistort<T>(camParams, distParams, mQ2T);

        Mat3 camMat;
        camMat << camParams[0], T(0), camParams[2], T(0), camParams[1], camParams[3], T(0), T(0),
            T(1);

        Mat3 transformation = camMat * homMat1.inverse() * homMat0 * camMat.inverse();
        symmetricReprojectionError<T>(transformation, mQ1T, mQ2T, errors, mWeight);
        return true;
    }

private:
    const Eigen::Vector2d mQ1;
    const Eigen::Vector2d mQ2;
    const double mWeight;
};
} // namespace ht
#endif // HABITRACK_ISOMETRY_GLOBAL_OPTIMIZER_H
