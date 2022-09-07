#include <cater/cache/basePairwiseCache.h>

namespace ct
{
BasePairwiseCache::BasePairwiseCache(
    std::size_t maxChunkSize, const std::vector<std::pair<std::size_t, std::size_t>>& pairs)
    : BaseCache(pairs.size(), maxChunkSize, {})
    , mPairs(pairs)
{
}

std::pair<std::size_t, std::size_t> BasePairwiseCache::getPair(std::size_t idx)
{
    return mPairs[idx];
}
} // namespace ct
