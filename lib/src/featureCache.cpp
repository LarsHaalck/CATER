#include "habitrack/featureCache.h"
#include "habitrack/baseFeatureContainer.h"

#include <iostream>

namespace ht
{
FeatureCache::FeatureCache(const BaseFeatureContainer& container, std::size_t numElems,
    std::size_t maxChunkSize, const std::vector<std::size_t>& ids)
    : BaseCache(numElems, maxChunkSize, ids)
    , mContainer(container)
{
}

std::vector<std::vector<cv::KeyPoint>> FeatureCache::getChunk(std::size_t idx)
{
    auto currSize = getChunkSize(idx);
    std::vector<std::vector<cv::KeyPoint>> cacheBlock;
    cacheBlock.reserve(currSize);

    const auto [lower, upper] = getChunkBounds(idx);
    for (std::size_t i = lower; i < upper; i++)
    {
        auto id = transformId(i);
        cacheBlock.push_back(mContainer.featureAt(id));
    }
    return cacheBlock;
}

std::unordered_map<std::size_t, std::vector<cv::KeyPoint>> FeatureCache::getChunkWithIdx(
    std::size_t idx)
{
    auto currSize = getChunkSize(idx);
    std::unordered_map<std::size_t, std::vector<cv::KeyPoint>> cacheBlock;
    cacheBlock.reserve(currSize);

    const auto [lower, upper] = getChunkBounds(idx);
    for (std::size_t i = lower; i < upper; i++)
    {
        auto id = transformId(i);
        cacheBlock.insert({id, mContainer.featureAt(id)});
    }
    return cacheBlock;
}

} // namespace ht
