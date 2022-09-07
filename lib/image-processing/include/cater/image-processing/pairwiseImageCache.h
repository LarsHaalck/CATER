#ifndef CATER_PAIRWISE_IMAGE_CACHE_H
#define CATER_PAIRWISE_IMAGE_CACHE_H

#include <cater/cache/basePairwiseCache.h>
#include <memory>
#include <opencv2/core.hpp>
#include <unordered_map>
#include <vector>

namespace ct
{
class BaseImageContainer;
}

namespace ct
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
#endif // CATER_PAIRWISE_IMAGE_CACHE_H
