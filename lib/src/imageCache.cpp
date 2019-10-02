#include "habitrack/imageCache.h"
#include "habitrack/imageContainer.h"

namespace ht
{
ImageCache::ImageCache(std::shared_ptr<ImageContainer> container,
    std::size_t maxChunkSize, bool useOnlyKeyFrames)
    : Cache(useOnlyKeyFrames ? container->getNumKeyFrames() : container->getNumImages(),
        maxChunkSize)
    , mContainer(std::move(container))
{
}

std::vector<cv::Mat> ImageCache::getChunk(std::size_t idx)
{
    auto currSize = getChunkSize(idx);
    std::vector<cv::Mat> cacheBlock;
    cacheBlock.reserve(currSize);

    for (std::size_t i = 0; i < currSize; i++)
        cacheBlock.push_back(mContainer->at(i));
    return cacheBlock;
}

} // namespace ht
