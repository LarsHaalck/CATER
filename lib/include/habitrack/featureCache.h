#ifndef HABITRACK_FEATURE_CACHE_H
#define HABITRACK_FEATURE_CACHE_H

#include <memory>
#include <vector>

#include <opencv2/core.hpp>

#include "baseCache.h"

namespace ht
{
    class FeatureContainer;
}

namespace ht
{
class FeatureCache : public BaseCache
{
public:
    FeatureCache(std::shared_ptr<FeatureContainer> container, std::size_t numElems,
        std::size_t maxChunkSize, const std::vector<std::size_t>& ids);
    std::vector<std::vector<cv::KeyPoint>> getChunk(std::size_t idx);
private:
    std::shared_ptr<FeatureContainer> mContainer;
};
}
#endif // HABITRACK_FEATURE_CACHE_H

