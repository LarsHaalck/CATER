#ifndef HABITRACK_IMAGE_CACHE_H
#define HABITRACK_IMAGE_CACHE_H

#include <memory>
#include <vector>

#include <opencv2/core.hpp>

#include "habitrack/cache.h"

namespace ht
{
    class ImageContainer;
}

namespace ht
{
class ImageCache : public Cache
{
public:
    ImageCache(std::shared_ptr<ImageContainer> container,
        std::size_t numElems, std::size_t maxChunkSize, bool useOnlyKeyFrames);
    std::vector<cv::Mat> getChunk(std::size_t idx);
private:
    std::shared_ptr<ImageContainer> mContainer;
};
}
#endif // HABITRACK_IMAGE_CACHE_H

