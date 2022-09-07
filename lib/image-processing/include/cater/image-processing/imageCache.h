#ifndef CATER_IMAGE_CACHE_H
#define CATER_IMAGE_CACHE_H

#include <cater/cache/baseCache.h>
#include <cater/image-processing/types.h>
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
class ImageCache : public BaseCache
{
public:
    ImageCache(const BaseImageContainer& container, std::size_t numElems, std::size_t maxChunkSize,
        const size_t_vec& ids);
    std::vector<cv::Mat> getChunk(std::size_t idx);
    std::unordered_map<std::size_t, cv::Mat> getChunkWithIdx(std::size_t idx);

private:
    cv::Mat getElem(std::size_t idx);

private:
    const BaseImageContainer& mContainer;
};
}
#endif // CATER_IMAGE_CACHE_H
