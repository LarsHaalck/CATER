#ifndef CATER_PAIR_RECOMMENDER_H
#define CATER_PAIR_RECOMMENDER_H

#include <vector>

namespace ct
{
class PairRecommender
{
public:
    PairRecommender() = default;
    virtual ~PairRecommender() { }
    virtual std::vector<std::pair<std::size_t, std::size_t>> getPairs(
        std::size_t size, std::size_t window, const std::vector<std::size_t>& ids)
        = 0;
};
} // namespace ct
#endif // CATER_PAIR_RECOMMENDER_H
