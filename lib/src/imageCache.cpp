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
    {
        auto id = transformId(i);
        cacheBlock.push_back(mContainer.at(id));
    }
    return cacheBlock;
}

std::unordered_map<std::size_t, cv::Mat> ImageCache::getChunkWithIdx(std::size_t idx)
{
    auto currSize = getChunkSize(idx);
    std::unordered_map<std::size_t, cv::Mat> cacheBlock;
    cacheBlock.reserve(currSize);

    auto [lower, upper] = getChunkBounds(idx);
    for (std::size_t i = lower; i < upper; i++)
    {
        auto id = transformId(i);
        cacheBlock.insert({id, mContainer.at(id)});
    }
    return cacheBlock;
}

} // namespace ht
