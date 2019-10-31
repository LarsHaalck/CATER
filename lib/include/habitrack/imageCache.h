#ifndef HABITRACK_IMAGE_CACHE_H
#define HABITRACK_IMAGE_CACHE_H

#include "habitrack/baseCache.h"
#include <memory>
#include <opencv2/core.hpp>
#include <vector>

namespace ht
{
class BaseImageContainer;
}

namespace ht
{
class ImageCache : public BaseCache
{
public:
    ImageCache(std::shared_ptr<BaseImageContainer> container, std::size_t numElems,
        std::size_t maxChunkSize, const std::vector<std::size_t>& ids);
    std::vector<cv::Mat> getChunk(std::size_t idx);

private:
    cv::Mat getElem(std::size_t idx);
private:
    std::shared_ptr<BaseImageContainer> mContainer;
};
}
#endif // HABITRACK_IMAGE_CACHE_H
