#ifndef CATER_SIMILARITY_GPS_OPTIMIZER_H
#define CATER_SIMILARITY_GPS_OPTIMIZER_H

#include <Eigen/Dense>

#include "metrics.h"

namespace ct
{
class SimilarityGPSFunctor
{
public:
    SimilarityGPSFunctor(const Eigen::Vector2d& gps);

    template <typename T>
    bool operator()(const T* const simVec, const T* const gpsVec, T* errors) const
    {
        using Mat3 = Eigen::Matrix<T, 3, 3, Eigen::RowMajor>;
        using Vec2 = Eigen::Matrix<T, 2, 1>;
        Mat3 gpsMat;
        gpsMat << gpsVec[0], gpsVec[1], gpsVec[2], -gpsVec[1], gpsVec[0], gpsVec[5], T(0),
            T(0), T(1);

        Vec2 mGPST(mGPS(0), mGPS(1));
        Vec2 mTransT(simVec[2], simVec[5]);

        symmetricReprojectionError<T>(gpsMat, mGPST, mTransT, errors, 1);
        return true;
    }

private:
    const Eigen::Vector2d mGPS;
};
} // namespace ct
#endif // CATER_SIMILARITY_GPS_OPTIMIZER_H
