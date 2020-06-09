#include "homographyGlobalOptimizer.h"

namespace ht
{
HomographyGlobalFunctor::HomographyGlobalFunctor(
    const Eigen::Vector2d& q1, const Eigen::Vector2d& q2, double weight)
    : mQ1(q1)
    , mQ2(q2)
    , mWeight(weight)
{
}
} // namespace ht