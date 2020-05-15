#ifndef HABITRACK_PAIRWISE_IMAGE_CACHE_H
#define HABITRACK_PAIRWISE_IMAGE_CACHE_H

#include <memory>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>

#include "cache/basePairwiseCache.h"

namespace ht
{
class BaseImageContainer;
}

namespace ht
{
class PairwiseImageCache : public BasePairwiseCache
{
public:
    PairwiseImageCache(const BaseImageContainer& container, std::size_t maxChunkSize,
        const std::vector<std::pair<std::size_t, std::size_t>>& pairs);

    std::unordered_map<std::size_t, cv::Mat> getChunk(std::size_t idx);

private:
    const BaseImageContainer& mContainer;
};
}
#endif // HABITRACK_PAIRWISE_IMAGE_CACHE_H
