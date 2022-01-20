#include <habitrack/image-processing/imageAggregator.h>

namespace ht
{
ImageAggregator::ImageAggregator(const std::vector<Images>& imgContainers)
    : mImgContainers(imgContainers)
    , mNumImgs()
    , mNumImgsVec()
{
    [[maybe_unused]] auto size = mImgContainers[0].getImgSize();
    assert(std::all_of(std::begin(mImgContainers), std::end(mImgContainers), [size](const auto& c) {
        return (c.getImgSize() == size);
    }) && "Image sizes of containers in ImageAggregator do not match");

    auto [total, sizeVec] = sumNumImgs();
    mNumImgs = total;
    mNumImgsVec = std::move(sizeVec);
}

std::pair<std::size_t, std::vector<std::size_t>> ImageAggregator::sumNumImgs() const
{
    std::vector<std::size_t> numVec;
    std::size_t total = 0;
    for (auto c : mImgContainers)
    {
        auto currSize = c.size();
        numVec.push_back(currSize);
        total += currSize;
    }
    return std::make_pair(total, numVec);
}

std::size_t ImageAggregator::size() const { return mNumImgs; }

cv::Mat ImageAggregator::at(std::size_t idx) const
{
    for (std::size_t i = 0; i < mImgContainers.size(); i++)
    {
        auto currSize = mNumImgsVec[i];
        if (idx < currSize)
            return mImgContainers[i].at(idx);

        idx -= currSize;
    }
    return cv::Mat();
}

cv::Size ImageAggregator::getImgSize() const { return mImgContainers[0].getImgSize(); }
cv::Point2f ImageAggregator::getCenter() const { return mImgContainers[0].getCenter(); }

ImageCache ImageAggregator::getCache(std::size_t maxChunkSize, const size_t_vec& ids) const
{
    return ImageCache {*this, size(), maxChunkSize, ids};
}

PairwiseImageCache ImageAggregator::getPairwiseCache(
    std::size_t maxChunkSize, const std::vector<std::pair<std::size_t, std::size_t>>& pairs) const
{
    return PairwiseImageCache(*this, maxChunkSize, pairs);
}
} // namespace ht
