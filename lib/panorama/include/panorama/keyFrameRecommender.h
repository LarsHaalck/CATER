#ifndef HABITRACK_KEYFRAME_RECOMMENDER_H
#define HABITRACK_KEYFRAME_RECOMMENDER_H

#include "image-processing/pairRecommender.h"

#include <memory>
#include <unordered_map>
#include <random>

namespace ht
{
class KeyFrameRecommender : public PairRecommender
{
public:
    KeyFrameRecommender(const std::vector<std::size_t>& keyFrames);

    std::vector<std::pair<std::size_t, std::size_t>> getPairs(
        std::size_t size, std::size_t window, const std::vector<std::size_t>& ids) override;

private:
    std::vector<std::size_t> mKeyFrames;
    std::random_device mRd;
    std::mt19937 mG;
};
} // namespace ht
#endif // HABITRACK_KEYFRAME_RECOMMENDER_H
