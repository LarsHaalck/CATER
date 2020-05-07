#include "habitrack/imageCache.h"
#include "habitrack/baseImageContainer.h"

namespace ht
{
ImageCache::ImageCache(const BaseImageContainer& container, std::size_t numElems,
    std::size_t maxChunkSize, const size_t_vec& ids)
    : BaseCache(numElems, maxChunkSize, ids)
    , mContainer(container)
{
}

std::vector<cv::Mat> ImageCache::getChunk(std::size_t idx)
{
    auto currSize = getChunkSize(idx);
    std::vector<cv::Mat> cacheBlock;
    cacheBlock.reserve(currSize);

    auto [lower, upper] = getChunkBounds(idx);
    for (std::size_t i = lower; i < upper; i++)
        cacheBlock.push_back(mContainer.at(transformId(i)));
    return cacheBlock;
}

} // namespace ht
