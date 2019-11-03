#ifndef HABITRACK_BASE_IMAGE_CONTAINER_H
#define HABITRACK_BASE_IMAGE_CONTAINER_H

#include "habitrack/imageCache.h"
#include <memory>
#include <vector>

namespace ht
{
using ImgId = std::size_t;
using ImgIds = std::vector<ImgId>;

class BaseImageContainer
{
public:
    BaseImageContainer() = default;

    virtual std::size_t getNumImgs() const = 0;
    virtual cv::Mat at(ImgId idx) const = 0;
    virtual cv::Size getImgSize() const = 0;

    virtual std::unique_ptr<ImageCache> getCache(
        std::size_t maxChunkSize, const ImgIds& ids = ImgIds())
        = 0;
};
} // namespace ht
#endif // HABITRACK_BASE_IMAGE_CONTAINER_H
