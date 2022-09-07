#ifndef CATER_BASE_IMAGE_CONTAINER_H
#define CATER_BASE_IMAGE_CONTAINER_H

#include <cater/image-processing/imageCache.h>
#include <cater/image-processing/pairwiseImageCache.h>
#include <cater/image-processing/types.h>
#include <memory>
#include <vector>

namespace ct
{
class BaseImageContainer
{
public:
    BaseImageContainer() = default;

    virtual std::size_t size() const = 0;
    virtual cv::Mat at(std::size_t idx) const = 0;
    virtual cv::Size getImgSize() const = 0;
    virtual cv::Point2f getCenter() const = 0;

    virtual ImageCache getCache(
        std::size_t maxChunkSize, const size_t_vec& ids = size_t_vec()) const = 0;
    virtual PairwiseImageCache getPairwiseCache(std::size_t maxChunkSize,
        const std::vector<std::pair<std::size_t, std::size_t>>& pairs) const = 0;
};
} // namespace ct
#endif // CATER_BASE_IMAGE_CONTAINER_H
