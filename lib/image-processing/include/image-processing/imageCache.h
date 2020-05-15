#ifndef HABITRACK_IMAGE_CACHE_H
#define HABITRACK_IMAGE_CACHE_H

#include <memory>
#include <opencv2/core.hpp>
#include <vector>
#include <unordered_map>

#include "cache/baseCache.h"
#include "image-processing/types.h"

namespace ht
{
class BaseImageContainer;
}

namespace ht
{
class ImageCache : public BaseCache
{
public:
    ImageCache(const BaseImageContainer& container, std::size_t numElems,
        std::size_t maxChunkSize, const size_t_vec& ids);
    std::vector<cv::Mat> getChunk(std::size_t idx);
    std::unordered_map<std::size_t, cv::Mat> getChunkWithIdx(std::size_t idx);

private:
    cv::Mat getElem(std::size_t idx);

private:
    const BaseImageContainer& mContainer;
};
}
#endif // HABITRACK_IMAGE_CACHE_H
