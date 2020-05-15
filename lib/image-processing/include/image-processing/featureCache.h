#ifndef HABITRACK_FEATURE_CACHE_H
#define HABITRACK_FEATURE_CACHE_H

#include "cache/baseCache.h"
#include <opencv2/core.hpp>
#include <unordered_map>
#include <vector>

namespace ht
{
class BaseFeatureContainer;
}

namespace ht
{
class FeatureCache : public BaseCache
{
public:
    FeatureCache(const BaseFeatureContainer& container, std::size_t numElems,
        std::size_t maxChunkSize, const std::vector<std::size_t>& ids);
    std::vector<std::vector<cv::KeyPoint>> getChunk(std::size_t idx);
    std::unordered_map<std::size_t, std::vector<cv::KeyPoint>> getChunkWithIdx(std::size_t idx);

private:
    const BaseFeatureContainer& mContainer;
};
}
#endif // HABITRACK_FEATURE_CACHE_H
