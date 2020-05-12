#include "habitrack/pairwiseImageCache.h"
#include "habitrack/baseImageContainer.h"

namespace ht
{
PairwiseImageCache::PairwiseImageCache(const BaseImageContainer& container,
    std::size_t maxChunkSize, const std::vector<std::pair<std::size_t, std::size_t>>& pairs)
    : BasePairwiseCache(maxChunkSize, pairs)
    , mContainer(container)
{
}

std::unordered_map<std::size_t, cv::Mat> PairwiseImageCache::getChunk(
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
            cacheBlock[idI] = mContainer.at(idI);

        if (!cacheBlock.count(idJ))
            cacheBlock[idJ] = mContainer.at(idJ);
    }
    return cacheBlock;
}

} // namespace ht
