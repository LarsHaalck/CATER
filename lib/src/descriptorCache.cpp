#include "habitrack/descriptorCache.h"
#include "habitrack/baseFeatureContainer.h"

namespace ht
{
DescriptorCache::DescriptorCache(const BaseFeatureContainer& container,
    std::size_t numElems, std::size_t maxChunkSize, const std::vector<std::size_t>& ids)
    : BaseCache(numElems, maxChunkSize, ids)
    , mContainer(container)
{
}

std::vector<cv::Mat> DescriptorCache::getChunk(std::size_t idx)
{
    auto currSize = getChunkSize(idx);
    std::vector<cv::Mat> cacheBlock;
    cacheBlock.reserve(currSize);

    const auto [lower, upper] = getChunkBounds(idx);
    for (std::size_t i = lower; i < upper; i++)
    {
        auto id = transformId(i);
        cacheBlock.push_back(mContainer.descriptorAt(transformId(id)));
    }
    return cacheBlock;
}

std::unordered_map<std::size_t, cv::Mat> DescriptorCache::getChunkWithIdx(std::size_t idx)
{
    auto currSize = getChunkSize(idx);
    std::unordered_map<std::size_t, cv::Mat> cacheBlock;
    cacheBlock.reserve(currSize);

    const auto [lower, upper] = getChunkBounds(idx);
    for (std::size_t i = lower; i < upper; i++)
    {
        auto id = transformId(i);
        cacheBlock.insert({id, mContainer.descriptorAt(transformId(id))});
    }
    return cacheBlock;
}

} // namespace ht
