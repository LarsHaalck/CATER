#include "habitrack/pairwiseDescriptorCache.h"
#include "habitrack/featureContainer.h"

#include "pairHash.h"

namespace ht
{
PairwiseDescriptorCache::PairwiseDescriptorCache(
    std::shared_ptr<FeatureContainer> container, std::size_t maxChunkSize,
    const std::vector<std::pair<std::size_t, std::size_t>>& pairs)
    : BasePairwiseCache(maxChunkSize, pairs)
    , mContainer(std::move(container))
{
}

std::unordered_map<std::size_t, cv::Mat> PairwiseDescriptorCache::getChunk(
    std::size_t idx)
{
    auto currSize = getChunkSize(idx);
    std::unordered_map<std::size_t, cv::Mat> cacheBlock;
    cacheBlock.reserve(2 * currSize);

    const auto [lower, upper] = getChunkBounds(idx);
    for (std::size_t i = lower; i < upper; i++)
    {
        auto [idI, idJ] = getPair(i);
        if (!cacheBlock.count(idI))
            cacheBlock[idI] = mContainer->descriptorAt(transformId(idI));

        if (!cacheBlock.count(idJ))
        cacheBlock[idJ] = mContainer->descriptorAt(transformId(idJ));
    }
    return cacheBlock;
}

} // namespace ht
