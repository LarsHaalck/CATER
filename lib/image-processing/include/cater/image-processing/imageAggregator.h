#ifndef CATER_IMAGE_AGGREGATOR_H
#define CATER_IMAGE_AGGREGATOR_H

#include <cater/image-processing/baseImageContainer.h>
#include <cater/image-processing/images.h>

namespace ct
{
class ImageAggregator : public BaseImageContainer
{
public:
    ImageAggregator(const std::vector<Images>& imgContainers);

    std::size_t size() const override;
    cv::Mat at(std::size_t idx) const override;
    cv::Size getImgSize() const override;
    cv::Point2f getCenter() const override;

    ImageCache getCache(
        std::size_t maxChunkSize, const size_t_vec& ids = size_t_vec()) const override;
    PairwiseImageCache getPairwiseCache(std::size_t maxChunkSize,
        const std::vector<std::pair<std::size_t, std::size_t>>& pairs) const override;

private:
    std::pair<std::size_t, std::vector<std::size_t>> sumNumImgs() const;

private:
    const std::vector<Images> mImgContainers;
    std::size_t mNumImgs;
    std::vector<std::size_t> mNumImgsVec;
};
} // namespace ct
#endif // CATER_IMAGE_AGGREGATOR_H
