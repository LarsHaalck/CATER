#ifndef HABITRACK_SIMILARITY_TEMP_OPTIMIZER_H
#define HABITRACK_SIMILARITY_TEMP_OPTIMIZER_H

#include <Eigen/Dense>
#include <ceres/jet.h>

namespace ht
{
class SimilarityTemporalConsistencyFunctor
{
public:
    SimilarityTemporalConsistencyFunctor() = default;

    template <typename T>
    bool operator()(
        const T* const simVec0, const T* const simVec1, const T* const simVec2, T* errors) const
    {
        using Mat3 = Eigen::Matrix<T, 3, 3, Eigen::RowMajor>;
        using Vec5 = Eigen::Matrix<T, 5, 1>;
        using namespace ceres;

        Mat3 homMat0;
        homMat0 << simVec0[0], simVec0[1], simVec0[2], -simVec0[1], simVec0[0], simVec0[5], T(0),
            T(0), T(1);

        Mat3 homMat1;
        homMat1 << simVec1[0], simVec1[1], simVec1[2], -simVec1[1], simVec1[0], simVec1[5], T(0),
            T(0), T(1);

        Mat3 homMat2;
        homMat2 << simVec2[0], simVec2[1], simVec2[2], -simVec2[1], simVec2[0], simVec2[5], T(0),
            T(0), T(1);

        /* T s0 = homMat0.col(0).norm(); */
        /* T s1 = homMat1.col(0).norm(); */

        /* // atan2 from ceres for T = ceres::Jet */
        /* T phi0 = atan2(homMat0(1, 0), homMat0(0, 0)); */
        /* T phi1 = atan2(homMat1(1, 0), homMat1(0, 0)); */

        /* errors[0] = s0 - s1; */
        /* errors[1] = cos(phi0) - cos(phi1); */
        /* errors[2] = sin(phi0) - sin(phi1); */
        /* errors[3] = static_cast<T>(homMat0(0, 2) - homMat1(0, 2)); // x */
        /* errors[4] = static_cast<T>(homMat0(1, 2) - homMat1(1, 2)); // y */
        /* return true; */

        T s0 = homMat0.col(0).norm();
        T s1 = homMat1.col(0).norm();
        T s2 = homMat2.col(0).norm();

        // atan2 from ceres for T = ceres::Jet
        T phi0 = atan2(homMat0(1, 0), homMat0(0, 0));
        T phi1 = atan2(homMat1(1, 0), homMat1(0, 0));
        T phi2 = atan2(homMat2(1, 0), homMat2(0, 0));

        Vec5 vec0;
        vec0 << s0, cos(phi0), sin(phi0), homMat0(0, 2), homMat0(1, 2);

        Vec5 vec1;
        vec1 << s1, cos(phi1), sin(phi1), homMat1(0, 2), homMat1(1, 2);

        Vec5 vec2;
        vec2 << s2, cos(phi2), sin(phi2), homMat2(0, 2), homMat2(1, 2);

        Vec5 vec21 = vec2 - vec1;
        /* vec21.normalize(); */
        Vec5 vec10 = vec1 - vec0;
        /* vec10.normalize(); */

        for (int i = 0; i < 5; i++)
            errors[i] = vec21(i) - vec10(i);
        return true;
    }
};
} // namespace ht
#endif // HABITRACK_SIMILARITY_TEMP_OPTIMIZER_H
