#ifndef HABITRACK_MILD_RECOMMENDER_H
#define HABITRACK_MILD_RECOMMENDER_H

#include "habitrack/pairRecommender.h"

#include <unordered_map>
#include <memory>

namespace ht
{
class FeatureContainer;
} // namespace ht

namespace ht
{
class MildRecommender : public PairRecommender
{
public:
    MildRecommender(std::shared_ptr<FeatureContainer> featureContainer);
    std::vector<std::pair<std::size_t, std::size_t>> getPairs(
        std::size_t size, std::size_t window, const std::vector<std::size_t>& ids);
private:
    void dilatePairList(std::unordered_map<std::pair<std::size_t, std::size_t>, double>& list,
        std::size_t size, std::size_t window) const;
private:
    std::shared_ptr<FeatureContainer> mFtContainer;

};
} // namespace ht
#endif // HABITRACK_MILD_RECOMMENDER_H
