#ifndef HABITRACK_BASE_IMAGE_CONTAINER_H
#define HABITRACK_BASE_IMAGE_CONTAINER_H

#include <memory>
#include <vector>

#include "habitrack/imageCache.h"
#include "habitrack/types.h"

namespace ht
{
class BaseImageContainer
{
public:
    BaseImageContainer() = default;

    virtual std::size_t size() const = 0;
    virtual cv::Mat at(std::size_t idx) const = 0;
    virtual cv::Size getImgSize() const = 0;

    virtual ImageCache getCache(
        std::size_t maxChunkSize, const size_t_vec& ids = size_t_vec()) const
        = 0;
};
} // namespace ht
#endif // HABITRACK_BASE_IMAGE_CONTAINER_H
