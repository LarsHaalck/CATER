#ifndef CATER_INTER_EXHAUSTIVE_RECOMMENDER_H
#define CATER_INTER_EXHAUSTIVE_RECOMMENDER_H

#include <cater/image-processing/pairRecommender.h>
#include <cater/util/pairHash.h>

#include <memory>
#include <unordered_map>

namespace ct
{
class Features;
class BaseFeatureContainer;
class FeatureAggregator;
class FeatureContainer;
} // namespace ct

namespace ct
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
} // namespace ct
#endif // CATER_INTER_EXHAUSTIVE_RECOMMENDER_H
