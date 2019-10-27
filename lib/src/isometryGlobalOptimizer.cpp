#include "isometryGlobalOptimizer.h"

namespace ht
{
IsometryGlobalFunctor::IsometryGlobalFunctor(const Eigen::Vector2d& q1,
    const Eigen::Vector2d& q2)
    : mQ1(q1)
    , mQ2(q2)
{
}
} // namespace ht
