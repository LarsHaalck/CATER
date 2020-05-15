#ifndef HABITRACK_PAIR_RECOMMENDER_H
#define HABITRACK_PAIR_RECOMMENDER_H

#include <vector>

namespace ht
{
class PairRecommender
{
public:
    PairRecommender() = default;
    virtual std::vector<std::pair<std::size_t, std::size_t>> getPairs(
        std::size_t size, std::size_t window, const std::vector<std::size_t>& ids)
        = 0;
};
} // namespace ht
#endif // HABITRACK_PAIR_RECOMMENDER_H
