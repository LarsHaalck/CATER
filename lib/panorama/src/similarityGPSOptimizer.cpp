#include "similarityGPSOptimizer.h"

namespace ct
{
SimilarityGPSFunctor::SimilarityGPSFunctor(const Eigen::Vector2d& gps)
    : mGPS(gps)
{
}
} // namespace ct
