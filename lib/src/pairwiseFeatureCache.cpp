#include "habitrack/pairwiseFeatureCache.h"
#include "habitrack/featureContainer.h"

#include "pairHash.h"

namespace ht
{
PairwiseFeatureCache::PairwiseFeatureCache(std::shared_ptr<FeatureContainer> container,
    std::size_t maxChunkSize, const std::vector<std::pair<std::size_t, std::size_t>>& pairs)
    : BasePairwiseCache(maxChunkSize, pairs)
    , mContainer(std::move(container))
{
}

std::unordered_map<std::size_t, std::vector<cv::KeyPoint>> PairwiseFeatureCache::getChunk(
    std::size_t idx)
{
    auto currSize = getChunkSize(idx);
    std::unordered_map<std::size_t, std::vector<cv::KeyPoint>> cacheBlock;
    cacheBlock.reserve(2 * currSize);

    const auto [lower, upper] = getChunkBounds(idx);
    for (std::size_t i = lower; i < upper; i++)
    {
        auto [idI, idJ] = getPair(i);
        if (!cacheBlock.count(idI))
            cacheBlock[idI] = mContainer->featureAt(idI);

        if (!cacheBlock.count(idJ))
            cacheBlock[idJ] = mContainer->featureAt(idJ);
    }
    return cacheBlock;
}

} // namespace ht
