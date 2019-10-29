#ifndef METRICS_H
#define METRICS_H

#include <Eigen/Dense>

template <typename T>
Eigen::Matrix<T, 2, 1> undistort(
    const T* const camParams, const T* const distParams, const Eigen::Matrix<T, 2, 1>& q)
{
    using Vec2 = Eigen::Matrix<T, 2, 1>;
    Vec2 qt;
    qt(0) = (q(0) - camParams[2]) / camParams[0];
    qt(1) = (q(1) - camParams[3]) / camParams[1];

    T r2 = qt(0) * qt(0) + qt(1) * qt(1);
    T r4 = r2 * r2;
    T r6 = r4 * r2;

    qt *= T(1) + distParams[0] * r2 + distParams[1] * r4 + distParams[2] * r6;
    /* qt *= T(1) + 0.2 * r2; */

    qt(0) = camParams[0] * qt(0) + camParams[2];
    qt(1) = camParams[1] * qt(1) + camParams[3];

    return qt;
}

template <typename T>
void symmetricReprojectionError(const Eigen::Matrix<T, 3, 3>& trafo,
    const Eigen::Matrix<T, 2, 1>& q1, const Eigen::Matrix<T, 2, 1>& q2, T errors[4],
    double weight)
{
    using Vec3 = Eigen::Matrix<T, 3, 1>;

    // calculate forward and backward projections after converting to homogeneous coords
    Vec3 q1Hom(q1(0), q1(1), T(1));
    Vec3 q2Hom(q2(0), q2(1), T(1));

    Vec3 trafoQ1 = trafo * q1.homogeneous();
    Vec3 trafoInvQ2 = trafo.inverse() * q2.homogeneous();

    // convert back to normalized coordinates
    trafoQ1 /= trafoQ1(2);
    trafoInvQ2 /= trafoInvQ2(2);

    // calculate elementwise errors
    errors[0] = weight * (trafoQ1(0) - q2(0));
    errors[1] = weight * (trafoQ1(1) - q2(1));
    errors[2] = weight * (trafoInvQ2(0) - q1(0));
    errors[3] = weight * (trafoInvQ2(1) - q1(1));
}

#endif // METRICS_H
