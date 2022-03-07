#ifndef HABITRACK_INTER_EXHAUSTIVE_RECOMMENDER_H
#define HABITRACK_INTER_EXHAUSTIVE_RECOMMENDER_H

#include <habitrack/image-processing/pairRecommender.h>
#include <habitrack/util/pairHash.h>

#include <memory>
#include <unordered_map>

namespace ht
{
class Features;
class BaseFeatureContainer;
class FeatureAggregator;
class FeatureContainer;
} // namespace ht

namespace ht
{
class InterExhaustiveRecommender : public PairRecommender
{
public:
    InterExhaustiveRecommender(const FeatureAggregator& ftContainers);

    std::vector<std::pair<std::size_t, std::size_t>> getPairs(
        std::size_t size, std::size_t window, const std::vector<std::size_t>& ids) override;

private:
    const BaseFeatureContainer& mFtContainers;
    std::vector<std::size_t> mBlockList;
};
} // namespace ht
#endif // HABITRACK_INTER_EXHAUSTIVE_RECOMMENDER_H
