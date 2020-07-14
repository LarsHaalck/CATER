#ifndef HABITRACK_MILD_RECOMMENDER_H
#define HABITRACK_MILD_RECOMMENDER_H

#include "image-processing/pairRecommender.h"

#include <memory>
#include <unordered_map>

namespace ht
{
class BaseFeatureContainer;
class FeatureAggregator;
class FeatureContainer;
} // namespace ht

namespace ht
{
class MildRecommender : public PairRecommender
{
public:
    explicit MildRecommender(
        const BaseFeatureContainer& ftContainer, int dilation = 0, bool addDiagonal = false);

    // TODO: fix
    /* MildRecommender(std::vector<std::shared_ptr<FeatureContainer>> ftContainers); */

    std::vector<std::pair<std::size_t, std::size_t>> getPairs(
        std::size_t size, std::size_t window, const std::vector<std::size_t>& ids) override;

private:
    void dilatePairList(std::unordered_map<std::pair<std::size_t, std::size_t>, double>& list,
        std::size_t size, std::size_t window) const;
    void addDiagonal(std::unordered_map<std::pair<std::size_t, std::size_t>, double>& list,
        std::size_t size, std::size_t window) const;

    void filterPairList(
        std::vector<std::pair<std::size_t, std::size_t>>& pairs, std::size_t size) const;

private:
    const BaseFeatureContainer& mFtContainer;
    std::vector<std::size_t> mBlockList;
    int mDilation;
    bool mDiag;
};
} // namespace ht
#endif // HABITRACK_MILD_RECOMMENDER_H
