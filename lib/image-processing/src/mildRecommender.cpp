#include "image-processing/mildRecommender.h"

#include "image-processing/featureAggregator.h"
#include "MILD/BayesianFilter.hpp"
#include "MILD/loop_closure_detector.h"
#include "image-processing/matches.h"
#include "image-processing/features.h"
#include <spdlog/spdlog.h>

namespace ht
{
MildRecommender::MildRecommender(
    const BaseFeatureContainer& ftContainer, int dilation, bool addDiagonal)
    : mFtContainer(ftContainer)
    , mBlockList()
    , mDilation(dilation / 2)
    , mDiag(addDiagonal)
{
    assert(mFtContainer.getFeatureType() == FeatureType::ORB
        && "FeatureType must be ORB for MILD Recommender");
}

MildRecommender::MildRecommender(
    const FeatureAggregator& ftContainers)
    : MildRecommender(ftContainers, 0, false)
{
    mBlockList = ftContainers.getBlockList();
}

std::vector<std::pair<std::size_t, std::size_t>> MildRecommender::getPairs(
    std::size_t, std::size_t window, const std::vector<std::size_t>& ids)
{
    // these parameters should not be changed
    MILD::LoopClosureDetector lcd(FEATURE_TYPE_ORB, 16, 0);

    // prob thes, non loop closure thresh, min shared thresh, min distance
    MILD::BayesianFilter filter(0.5, 2, 2, window);

    Eigen::VectorXf prevVisitProb(1);
    prevVisitProb << 0.1;
    std::vector<Eigen::VectorXf> prevVisitFlag;

    /* std::vector<std::pair<std::size_t, std::size_t>> pairs; */
    std::unordered_map<std::pair<std::size_t, std::size_t>, double> scores;

    auto transId = [&ids](std::size_t i) { return ids.empty() ? i : ids[i]; };

    std::size_t numImgs = ids.empty() ? mFtContainer.size() : ids.size();
    spdlog::info("Using MILD to get possible image pairs");
    for (std::size_t k = 0; k < numImgs; k++)
    {
        auto desc = mFtContainer.descriptorAt(transId(k));

        std::vector<float> simScore;
        simScore.clear();
        lcd.insert_and_query_database(desc, simScore);
        filter.filter(simScore, prevVisitProb, prevVisitFlag);

        if (prevVisitFlag.size() >= 1)
        {
            for (int i = 0; i < prevVisitFlag[prevVisitFlag.size() - 1].size(); i++)
            {
                scores[std::make_pair(i, k)] = prevVisitFlag[prevVisitFlag.size() - 1][i];
            }
        }
    }

    if (prevVisitFlag.size() >= 4)
    {
        for (int j = 0; j < 3; j++)
        {
            for (int i = 0; i < prevVisitFlag[prevVisitFlag.size() - (3 - j)].size(); i++)
            {
                scores[std::make_pair(i, numImgs - (3 - j))]
                    = prevVisitFlag[prevVisitFlag.size() - (3 - j)][i];
            }
        }
    }

    dilatePairList(scores, numImgs, window);
    if (mDiag)
        addDiagonal(scores, numImgs, window);

    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    for (const auto& [pair, score] : scores)
    {
        if (score > 0)
            pairs.push_back(std::make_pair(transId(pair.first), transId(pair.second)));
    }

    filterPairList(pairs, numImgs);
    return pairs;
}

void MildRecommender::dilatePairList(
    std::unordered_map<std::pair<std::size_t, std::size_t>, double>& list, std::size_t size,
    std::size_t window) const
{
    auto w = mDilation;
    auto pairs = matches::getKeyList(list);
    for (const auto& [i, j] : pairs)
    {
        if (list.at({i, j}) > 0)
        {
            for (int n = -w; n <= w; n++)
            {
                for (int m = -w; m <= w; m++)
                {
                    auto iShift = i + n;
                    auto jShift = j + m;
                    auto pairShift = std::make_pair(iShift, jShift);
                    // check boundary conditions and if the key already exists, otherwise insert
                    if (iShift < size && jShift < size && jShift > iShift + window
                        && !list.count(pairShift))
                    {
                        list.insert(std::make_pair(pairShift, 1.0));
                    }
                }
            }
        }
    }
}

void MildRecommender::addDiagonal(
    std::unordered_map<std::pair<std::size_t, std::size_t>, double>& list, std::size_t size,
    std::size_t window) const
{
    for (std::size_t i = 0; i < size; i++)
    {
        for (std::size_t j = i + 1; (j < i + window) && (j < size); j++)
        {
            auto pair = std::make_pair(i, j);
            if (!list.count(pair))
                list.insert(std::make_pair(pair, 1.0));
        }
    }
}

// is only called for inter video loop closure
void MildRecommender::filterPairList(
    std::vector<std::pair<std::size_t, std::size_t>>& pairs, std::size_t) const
{
    if (mBlockList.empty())
        return;

    {
        std::sort(std::begin(pairs), std::end(pairs));

        std::size_t currBlock = 0;
        std::size_t shift = 0;
        std::vector<std::pair<std::size_t, std::size_t>> keepPairs;
        for (std::size_t i = 0; i < pairs.size(); i++)
        {
            auto pair = pairs[i];
            if (pair.first - shift >= mBlockList[currBlock])
                shift += mBlockList[currBlock++];

            // delete intra video matches
            // keep only matches from one video to another
            if (pair.second - shift >= mBlockList[currBlock])
                keepPairs.push_back(pair);
            /* else */
            /*     std::cout << pair.first << ", " << pair.second << std::endl; */
        }
        pairs = std::move(keepPairs);
    }

    // add pairs for easy fusing between start and end points of video
    /* for (std::size_t i = 0; i < mBlockList.size(); i++) */
    /* { */
    /*     auto currSize = mBlockList[i]; */
    /*     for (int n = -5; n <= 5; n++) */
    /*     { */
    /*         auto idI = currSize + n; */
    /*         for (int m = n + 1; m <= 5; m++) */
    /*         { */
    /*             auto idJ = currSize + m; */
    /*             if (idI < size && idJ < size) */
    /*             { */
    /*                 auto pair = std::make_pair(idI, idJ); */
    /*                 if (!list.count(pair)) */
    /*                     list.insert(std::make_pair(pair, 1.0)); */
    /*             } */
    /*         } */
    /*     } */
    /* } */
}

} // namespace ht
