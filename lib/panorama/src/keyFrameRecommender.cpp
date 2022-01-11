#include "panorama/keyFrameRecommender.h"

#include <numeric>
#include <algorithm>

namespace ht
{
KeyFrameRecommender::KeyFrameRecommender(const std::vector<std::size_t>& keyFrames)
    : mKeyFrames(keyFrames)
    , mRd()
    , mG(mRd())
{
}

// only kf to non-kf
/* std::vector<std::pair<std::size_t, std::size_t>> KeyFrameRecommender::getPairs( */
/*     std::size_t, std::size_t, const std::vector<std::size_t>&) */
/* { */
/*     std::vector<std::pair<std::size_t, std::size_t>> pairs; */

/*     for (std::size_t i = 1; i < mKeyFrames.size(); i++) */
/*     { */
/*         auto prevView = mKeyFrames[i - 1]; */
/*         auto currView = mKeyFrames[i]; */

/*         for (std::size_t j = prevView + 1; j < currView; j++) */
/*         { */
/*             pairs.push_back(std::make_pair(prevView, j)); */
/*             pairs.push_back(std::make_pair(j, currView)); */
/*         } */
/*     } */
/*     return pairs; */
/* } */

// kf to non-kf + chain
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

            // also generate a chain through all frames to reg. smoothnes
            if (j > prevView + 1)
                pairs.push_back(std::make_pair(j - 1, j));
        }
    }
    return pairs;
}

// full extensive (not feasable)
/* std::vector<std::pair<std::size_t, std::size_t>> KeyFrameRecommender::getPairs( */
/*     std::size_t, std::size_t, const std::vector<std::size_t>&) */
/* { */
/*     std::vector<std::pair<std::size_t, std::size_t>> pairs; */

/*     for (std::size_t i = 1; i < mKeyFrames.size(); i++) */
/*     { */
/*         auto prevView = mKeyFrames[i - 1]; */
/*         auto currView = mKeyFrames[i]; */

/*         for (std::size_t j = prevView; j < currView; j++) */
/*         { */
/*             for (std::size_t k = j + 1; k <= currView; k++) */
/*                 pairs.push_back(std::make_pair(j, k)); */
/*         } */
/*     } */
/*     return pairs; */
/* } */

// kf to non-kf + remainings/10 randoms
/* std::vector<std::pair<std::size_t, std::size_t>> KeyFrameRecommender::getPairs( */
/*     std::size_t, std::size_t, const std::vector<std::size_t>&) */
/* { */
/*     std::vector<std::pair<std::size_t, std::size_t>> pairs; */

/*     for (std::size_t i = 1; i < mKeyFrames.size(); i++) */
/*     { */
/*         auto prevView = mKeyFrames[i - 1]; */
/*         auto currView = mKeyFrames[i]; */

/*         for (std::size_t j = prevView + 1; j < currView; j++) */
/*         { */
/*             pairs.push_back(std::make_pair(prevView, j)); */
/*             pairs.push_back(std::make_pair(j, currView)); */

/*             std::vector<std::size_t> remaining(currView - (j + 1)); */
/*             std::iota(std::begin(remaining), std::end(remaining), j + 1); */
/*             std::shuffle(std::begin(remaining), std::end(remaining), mG); */
/*             for (std::size_t k = 0; k < remaining.size() / 100; k++) */
/*                 pairs.push_back(std::make_pair(j, remaining[k])); */
/*         } */
/*     } */
/*     return pairs; */
/* } */
}
