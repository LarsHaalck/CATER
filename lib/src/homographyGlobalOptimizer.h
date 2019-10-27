#ifndef HABITRACK_HOMOGRAPHY_GLOBAL_OPTIMIZER_H
#define HABITRACK_HOMOGRAPHY_GLOBAL_OPTIMIZER_H

#include "metrics.h"
#include <Eigen/Dense>

namespace ht
{
class HomographyGlobalFunctor
{
public:
    HomographyGlobalFunctor(const Eigen::Vector2d& q1, const Eigen::Vector2d& q2);

    template <typename T>
    bool operator()(const T* const camParams, const T* const distParams, const T* const homVec0,
        const T* const homVec1, T* errors) const
    {
        using Mat3 = Eigen::Matrix<T, 3, 3, Eigen::RowMajor>;
        using Vec2 = Eigen::Matrix<T, 2, 1>;

        Mat3 homMat0(homVec0);
        Mat3 homMat1(homVec1);

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
}
#endif // HABITRACK_HOMOGRAPHY_GLOBAL_OPTIMIZER_H
