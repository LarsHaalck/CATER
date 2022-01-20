#include <habitrack/image-processing/featureAggregator.h>

namespace ht
{
FeatureAggregator::FeatureAggregator(const std::vector<Features>& ftContainers)
    : mFtContainers(ftContainers)
    , mNumImgs()
    , mNumImgsVec()
{
    if (ftContainers.empty())
        return;

    [[maybe_unused]] auto size = mFtContainers[0].getImageSize();
    [[maybe_unused]] auto ftType = mFtContainers[0].getFeatureType();
    for ([[maybe_unused]] const auto& c : mFtContainers)
    {
        assert(c.getImageSize() == size
            && "image sizes of containers in FeatureAggregator do not match");
        assert(c.getFeatureType() == ftType
            && "feature types of containers in FeatureAggregator do not match");
    }

    auto [total, sizeVec] = sumNumImgs(ftContainers);
    mNumImgs = total;
    mNumImgsVec = std::move(sizeVec);
}

FeatureAggregator::FeatureAggregator(const Features& fts)
    : FeatureAggregator(std::vector<Features> {fts})
{
}

std::pair<std::size_t, std::vector<std::size_t>> FeatureAggregator::sumNumImgs(
    const std::vector<Features>& fts) const
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
            return mFtContainers[i].featureAt(idx);

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
            return mFtContainers[i].descriptorAt(idx);

        idx -= currSize;
    }
    return cv::Mat();
}

FeatureCache FeatureAggregator::getFeatureCache(
    std::size_t maxChunkSize, const size_t_vec& ids) const
{
    auto numElems = size();
    return FeatureCache(*this, numElems, maxChunkSize, ids);
}

DescriptorCache FeatureAggregator::getDescriptorCache(
    std::size_t maxChunkSize, const size_t_vec& ids) const
{
    auto numElems = size();
    return DescriptorCache(*this, numElems, maxChunkSize, ids);
}

PairwiseDescriptorCache FeatureAggregator::getPairwiseDescriptorCache(
    std::size_t maxChunkSize, const std::vector<std::pair<std::size_t, std::size_t>>& pairs) const
{
    return PairwiseDescriptorCache(*this, maxChunkSize, pairs);
}

PairwiseFeatureCache FeatureAggregator::getPairwiseFeatureCache(
    std::size_t maxChunkSize, const std::vector<std::pair<std::size_t, std::size_t>>& pairs) const
{
    return PairwiseFeatureCache(*this, maxChunkSize, pairs);
}

std::size_t FeatureAggregator::size() const { return mNumImgs; }

cv::Size FeatureAggregator::getImageSize() const { return mFtContainers[0].getImageSize(); }

FeatureType FeatureAggregator::getFeatureType() const { return mFtContainers[0].getFeatureType(); }

std::vector<std::size_t> FeatureAggregator::getBlockList() const
{
    std::vector<std::size_t> blockList;

    for (const auto& ftContainer : mFtContainers)
        blockList.push_back(ftContainer.size());

    return blockList;
}

} // namespace ht
