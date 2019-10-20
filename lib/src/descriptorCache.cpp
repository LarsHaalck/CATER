#include "habitrack/descriptorCache.h"
#include "habitrack/featureContainer.h"

#include <iostream>

namespace ht
{
DescriptorCache::DescriptorCache(std::shared_ptr<FeatureContainer> container,
    std::size_t numElems, std::size_t maxChunkSize, ImageType imageType)
    : Cache(numElems, maxChunkSize, imageType)
    , mContainer(std::move(container))
{
}

std::vector<cv::Mat> DescriptorCache::getChunk(std::size_t idx)
{
    auto currSize = getChunkSize(idx);
    std::vector<cv::Mat> cacheBlock;
    cacheBlock.reserve(currSize);

    auto [lower, upper] = getChunkBounds(idx);
    for (std::size_t i = lower; i < upper; i++)
        cacheBlock.push_back(mContainer->descriptorAt(i, mImageType));
    return cacheBlock;
}

} // namespace ht
