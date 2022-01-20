#ifndef HABITRACK_DESCRIPTOR_CACHE_H
#define HABITRACK_DESCRIPTOR_CACHE_H

#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>

#include <habitrack/cache/baseCache.h>

namespace ht
{
class BaseFeatureContainer;
}

namespace ht
{
class DescriptorCache : public BaseCache
{
public:
    DescriptorCache(const BaseFeatureContainer& container, std::size_t numElems,
        std::size_t maxChunkSize, const std::vector<std::size_t>& ids);
    std::vector<cv::Mat> getChunk(std::size_t idx);
    std::unordered_map<std::size_t, cv::Mat> getChunkWithIdx(std::size_t idx);

private:
    const BaseFeatureContainer& mContainer;
};
}
#endif // HABITRACK_DESCRIPTOR_CACHE_H
