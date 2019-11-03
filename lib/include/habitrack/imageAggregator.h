#ifndef HABITRACK_IMAGE_AGGREGATOR_H
#define HABITRACK_IMAGE_AGGREGATOR_H

#include "habitrack/baseImageContainer.h"
#include "habitrack/imageContainer.h"

namespace ht
{
class ImageAggregator : public BaseImageContainer,
                        public std::enable_shared_from_this<ImageAggregator>
{
public:
    ImageAggregator(const std::vector<std::shared_ptr<ImageContainer>>& imgContainers);
    ImageAggregator(std::shared_ptr<ImageContainer> imgContainer);

    std::size_t getNumImgs() const override;
    cv::Mat at(ImgId idx) const override;
    cv::Size getImgSize() const override;

    std::unique_ptr<ImageCache> getCache(
        std::size_t maxChunkSize, const ImgIds& ids = ImgIds()) override;

private:
    std::pair<std::size_t, std::vector<std::size_t>> sumNumImgs(
        const std::vector<std::shared_ptr<ImageContainer>>& imgContainers) const;

private:
    std::vector<std::shared_ptr<ImageContainer>> mImgContainers;
    std::size_t mNumImgs;
    std::vector<std::size_t> mNumImgsVec;
};
} // namespace ht
#endif // HABITRACK_IMAGE_AGGREGATOR_H
