#include "habitrack/featureCache.h"
#include "habitrack/featureContainer.h"

#include <iostream>

namespace ht
{
FeatureCache::FeatureCache(std::shared_ptr<FeatureContainer> container, std::size_t numElems,
    std::size_t maxChunkSize, const std::vector<std::size_t>& ids)
    : BaseCache(numElems, maxChunkSize, ids)
    , mContainer(std::move(container))
{
}

std::vector<std::vector<cv::KeyPoint>> FeatureCache::getChunk(std::size_t idx)
{
    auto currSize = getChunkSize(idx);
    std::vector<std::vector<cv::KeyPoint>> cacheBlock;
    cacheBlock.reserve(currSize);

    const auto [lower, upper] = getChunkBounds(idx);
    for (std::size_t i = lower; i < upper; i++)
        cacheBlock.push_back(mContainer->featureAt(transformId(i)));
    return cacheBlock;
}

} // namespace ht
