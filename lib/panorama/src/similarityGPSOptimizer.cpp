#include "similarityGPSOptimizer.h"

namespace ht
{
SimilarityGPSFunctor::SimilarityGPSFunctor(const Eigen::Vector2d& gps)
    : mGPS(gps)
{
}
} // namespace ht
