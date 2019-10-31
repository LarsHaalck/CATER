#include "habitrack/imageAggregator.h"

namespace ht
{
ImageAggregator::ImageAggregator(const std::vector<std::shared_ptr<ImageContainer>>& imgContainers)
    : mImgContainers(imgContainers)
    , mNumImgs()
    , mNumImgsVec()
{
    auto size = mImgContainers[0]->getImgSize();
    for (auto& c : mImgContainers)
    {
        assert(c->getImgSize() == size
            && "Image sizes of containers in ImageAggregator do not match");
    }

    auto [total, sizeVec] = sumNumImgs(imgContainers);
    mNumImgs = total;
    mNumImgsVec = std::move(sizeVec);
}

ImageAggregator::ImageAggregator(std::shared_ptr<ImageContainer> imgContainer)
    : ImageAggregator(std::vector<std::shared_ptr<ImageContainer>>{imgContainer})
{
}

std::pair<std::size_t, std::vector<std::size_t>> ImageAggregator::sumNumImgs(
    const std::vector<std::shared_ptr<ImageContainer>>& imgContainers) const
{
    std::vector<std::size_t> numVec;
    std::size_t total = 0;
    for (auto& c : imgContainers)
    {
        auto currSize = c->getNumImgs();
        numVec.push_back(currSize);
        total += currSize;

    }
    return std::make_pair(total, numVec);
}

std::size_t ImageAggregator::getNumImgs() const { return mNumImgs; }

cv::Mat ImageAggregator::at(ImgId idx) const
{
    for (std::size_t i = 0; i < mImgContainers.size(); i++)
    {
        auto currSize = mNumImgsVec[i];
        if (idx < currSize)
            return mImgContainers[i]->at(idx);

        idx -= currSize;
    }
    return cv::Mat();
}

cv::Size ImageAggregator::getImgSize() const { return mImgContainers[0]->getImgSize(); }

std::unique_ptr<ImageCache> ImageAggregator::getCache(std::size_t maxChunkSize, const ImgIds& ids)
{
    auto numElems = getNumImgs();
    return std::make_unique<ImageCache>(shared_from_this(), numElems, maxChunkSize, ids);
}
} // namespace ht
