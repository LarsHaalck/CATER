#ifndef CATER_KEYFRAME_RECOMMENDER_H
#define CATER_KEYFRAME_RECOMMENDER_H

#include <cater/image-processing/pairRecommender.h>

#include <memory>
#include <unordered_map>

namespace ct
{
class KeyFrameRecommender : public PairRecommender
{
public:
    KeyFrameRecommender(const std::vector<std::size_t>& keyFrames);

    std::vector<std::pair<std::size_t, std::size_t>> getPairs(
        std::size_t size, std::size_t window, const std::vector<std::size_t>& ids) override;

private:
    std::vector<std::size_t> mKeyFrames;
};
} // namespace ct
#endif // CATER_KEYFRAME_RECOMMENDER_H
