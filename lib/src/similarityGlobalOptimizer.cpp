#include "similarityGlobalOptimizer.h"

namespace ht
{
SimilarityGlobalFunctor::SimilarityGlobalFunctor(
    double weight, const Eigen::Vector2d& q1, const Eigen::Vector2d& q2)
    : mWeight(weight)
    , mQ1(q1)
    , mQ2(q2)
{
}
} // namespace ht
