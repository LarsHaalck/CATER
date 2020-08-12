#include "image-processing/featureAggregator.h"

namespace ht
{
FeatureAggregator::FeatureAggregator(const std::vector<const Features&>& ftContainers)
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

FeatureAggregator::FeatureAggregator(const Features& fts)
    : FeatureAggregator(std::vector<const Features&> {fts})
{
}

std::pair<std::size_t, std::vector<std::size_t>> FeatureAggregator::sumNumImgs(
    const std::vector<const Features&>& fts) const
{
    std::vector<std::size_t> numVec;
    std::size_t total = 0;
    for (const auto& c : fts)
    {
        auto currSize = c.size();
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

FeatureCache FeatureAggregator::getFeatureCache(std::size_t maxChunkSize, const size_t_vec& ids)
{
    auto numElems = size();
    return FeatureCache(*this, numElems, maxChunkSize, ids);
}

DescriptorCache FeatureAggregator::getDescriptorCache(std::size_t maxChunkSize, const ImgIds& ids)
{
    auto numElems = size();
    return DescriptorCache(*this, numElems, maxChunkSize, ids);
}

PairwiseDescriptorCache FeatureAggregator::getPairwiseDescriptorCache(
    std::size_t maxChunkSize, const std::vector<std::pair<std::size_t, std::size_t>>& pairs)
{
    return PairwiseDescriptorCache(*this, maxChunkSize, pairs);
}

PairwiseFeatureCache FeatureAggregator::getPairwiseFeatureCache(
    std::size_t maxChunkSize, const std::vector<std::pair<std::size_t, std::size_t>>& pairs)
{
    return PairwiseFeatureCache(*this, maxChunkSize, pairs);
}

std::size_t FeatureAggregator::size() const { return mNumImgs; }
cv::Size FeatureAggregator::getImageSize() const { return mFtContainers[0]->getImgSize(); }
FeatureType FeatureAggregator::getFeatureType() const { return mFtContainers[0]->getFtType(); }

} // namespace ht
