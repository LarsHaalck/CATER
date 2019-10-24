#include "habitrack/descriptorCache.h"
#include "habitrack/featureContainer.h"

namespace ht
{
DescriptorCache::DescriptorCache(std::shared_ptr<FeatureContainer> container, std::size_t numElems,
    std::size_t maxChunkSize, const std::vector<std::size_t>& ids)
    : BaseCache(numElems, maxChunkSize, ids)
    , mContainer(std::move(container))
{
}

std::vector<cv::Mat> DescriptorCache::getChunk(std::size_t idx)
{
    auto currSize = getChunkSize(idx);
    std::vector<cv::Mat> cacheBlock;
    cacheBlock.reserve(currSize);

    const auto [lower, upper] = getChunkBounds(idx);
    for (std::size_t i = lower; i < upper; i++)
        cacheBlock.push_back(mContainer->descriptorAt(transformId(i)));
    return cacheBlock;
}

} // namespace ht
