#include "image-processing/featureAggregator.h"

namespace ht
{
FeatureAggregator::FeatureAggregator(
    const std::vector<std::shared_ptr<FeatureContainer>>& ftContainers)
    : mFtContainers(ftContainers)
    , mNumImgs()
    , mNumImgsVec()
{
    auto size = mFtContainers[0]->getImgSize();
    auto ftType = mFtContainers[0]->getFtType();
    for (auto& c : mFtContainers)
    {
        assert(c->getImgSize() == size
            && "image sizes of containers in FeatureAggregator do not match");
        assert(c->getFtType() == ftType
            && "feature types of containers in FeatureAggregator do not match");
    }

    auto [total, sizeVec] = sumNumImgs(ftContainers);
    mNumImgs = total;
    mNumImgsVec = std::move(sizeVec);
}

FeatureAggregator::FeatureAggregator(std::shared_ptr<FeatureContainer> imgContainer)
    : FeatureAggregator(std::vector<std::shared_ptr<FeatureContainer>> {imgContainer})
{
}

std::pair<std::size_t, std::vector<std::size_t>> FeatureAggregator::sumNumImgs(
    const std::vector<std::shared_ptr<FeatureContainer>>& imgContainers) const
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

std::vector<cv::KeyPoint> FeatureAggregator::featureAt(std::size_t idx) const
{
    for (std::size_t i = 0; i < mFtContainers.size(); i++)
    {
        auto currSize = mNumImgsVec[i];
        if (idx < currSize)
            return mFtContainers[i]->featureAt(idx);

        idx -= currSize;
    }
    return {};
}

cv::Mat FeatureAggregator::descriptorAt(std::size_t idx) const
{
    for (std::size_t i = 0; i < mFtContainers.size(); i++)
    {
        auto currSize = mNumImgsVec[i];
        if (idx < currSize)
            return mFtContainers[i]->descriptorAt(idx);

        idx -= currSize;
    }
    return cv::Mat();
}

std::unique_ptr<FeatureCache> FeatureAggregator::getFeatureCache(
    std::size_t maxChunkSize, const ImgIds& ids)
{
    auto numElems = getNumImgs();
    return std::make_unique<FeatureCache>(shared_from_this(), numElems, maxChunkSize, ids);
}

std::unique_ptr<DescriptorCache> FeatureAggregator::getDescriptorCache(
    std::size_t maxChunkSize, const ImgIds& ids)
{
    auto numElems = getNumImgs();
    return std::make_unique<DescriptorCache>(shared_from_this(), numElems, maxChunkSize, ids);
}

std::unique_ptr<PairwiseDescriptorCache> FeatureAggregator::getPairwiseDescriptorCache(
    std::size_t maxChunkSize, const std::vector<std::pair<std::size_t, std::size_t>>& pairs)
{
    return std::make_unique<PairwiseDescriptorCache>(shared_from_this(), maxChunkSize, pairs);
}

std::unique_ptr<PairwiseFeatureCache> FeatureAggregator::getPairwiseFeatureCache(
    std::size_t maxChunkSize, const std::vector<std::pair<std::size_t, std::size_t>>& pairs)
{
    return std::make_unique<PairwiseFeatureCache>(shared_from_this(), maxChunkSize, pairs);
}

std::size_t FeatureAggregator::getNumImgs() const { return mNumImgs; }
cv::Size FeatureAggregator::getImgSize() const { return mFtContainers[0]->getImgSize(); }
FeatureType FeatureAggregator::getFtType() const { return mFtContainers[0]->getFtType(); }

} // namespace ht
