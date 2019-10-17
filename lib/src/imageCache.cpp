#include "habitrack/imageCache.h"
#include "habitrack/imageContainer.h"

#include <iostream>

namespace ht
{
ImageCache::ImageCache(std::shared_ptr<ImageContainer> container,
    std::size_t numElems, std::size_t maxChunkSize, bool keyFramesOnly)
    : Cache(numElems, maxChunkSize, keyFramesOnly)
    , mContainer(std::move(container))
{
}

std::vector<cv::Mat> ImageCache::getChunk(std::size_t idx)
{
    auto currSize = getChunkSize(idx);
    std::vector<cv::Mat> cacheBlock;
    cacheBlock.reserve(currSize);

    auto [lower, upper] = getChunkBounds(idx);
    for (std::size_t i = lower; i < upper; i++)
        cacheBlock.push_back(mContainer->at(i, mKeyFramesOnly));
    return cacheBlock;
}

} // namespace ht
