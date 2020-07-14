#include "panorama/keyFrameRecommender.h"

namespace ht
{
KeyFrameRecommender::KeyFrameRecommender(const std::vector<std::size_t>& keyFrames)
    : mKeyFrames(keyFrames)
{
}

std::vector<std::pair<std::size_t, std::size_t>> KeyFrameRecommender::getPairs(
    std::size_t, std::size_t, const std::vector<std::size_t>&)
{
    std::vector<std::pair<std::size_t, std::size_t>> pairs;

    for (std::size_t i = 1; i < mKeyFrames.size(); i++)
    {
        auto prevView = mKeyFrames[i - 1];
        auto currView = mKeyFrames[i];

        for (std::size_t j = prevView + 1; j < currView; j++)
        {
            pairs.push_back(std::make_pair(prevView, j));
            pairs.push_back(std::make_pair(j, currView));
        }
    }
    return pairs;
}
}
