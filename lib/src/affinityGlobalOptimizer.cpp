#include "affinityGlobalOptimizer.h"

namespace ht
{
AffinityGlobalFunctor::AffinityGlobalFunctor(const Eigen::Vector2d& q1,
    const Eigen::Vector2d& q2)
    : mQ1(q1)
    , mQ2(q2)
{
}
} // namespace ht
