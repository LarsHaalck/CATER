#include "habitrack/imageCache.h"
#include "habitrack/baseImageContainer.h"

namespace ht
{
ImageCache::ImageCache(std::shared_ptr<BaseImageContainer> container, std::size_t numElems,
    std::size_t maxChunkSize, const ImgIds& ids)
    : BaseCache(numElems, maxChunkSize, ids)
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
        cacheBlock.push_back(mContainer->at(transformId(i)));
    return cacheBlock;
}

} // namespace ht
