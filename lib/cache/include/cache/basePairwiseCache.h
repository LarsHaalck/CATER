#ifndef HABITRACK_BASE_PAIRWISE_BASE_CACHE_H
#define HABITRACK_BASE_PAIRWISE_BASE_CACHE_H

#include <cstddef>
#include <tuple>
#include <vector>

#include "cache/baseCache.h"

namespace ht
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
} // namespace ht

#endif // HABITRACK_BASE_PAIRWISE_BASE_CACHE_H
