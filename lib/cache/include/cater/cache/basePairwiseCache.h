#ifndef CATER_BASE_PAIRWISE_BASE_CACHE_H
#define CATER_BASE_PAIRWISE_BASE_CACHE_H

#include <cater/cache/baseCache.h>
#include <cstddef>
#include <tuple>
#include <vector>

namespace ct
{
class BasePairwiseCache : public BaseCache
{
public:
    BasePairwiseCache(
        std::size_t maxChunkSize, const std::vector<std::pair<std::size_t, std::size_t>>& pairs);

protected:
    std::pair<std::size_t, std::size_t> getPair(std::size_t i);

private:
    std::vector<std::pair<std::size_t, std::size_t>> mPairs;
};
} // namespace ct

#endif // CATER_BASE_PAIRWISE_BASE_CACHE_H
