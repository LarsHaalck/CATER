#include "habitrack/mildRecommender.h"
#include "habitrack/featureAggregator.h"
#include "habitrack/matchesContainer.h"

#include "MILD/BayesianFilter.hpp"
#include "MILD/loop_closure_detector.h"

#include "progressBar.h"

namespace ht
{
MildRecommender::MildRecommender(std::shared_ptr<BaseFeatureContainer> ftContainer)
    : mFtContainer(std::move(ftContainer))
    , mBlockList()
{
    assert(mFtContainer->getFtType() == FeatureType::ORB &&
        "FeatureType must be ORB for MILD Recommender");

}

MildRecommender::MildRecommender(std::vector<std::shared_ptr<FeatureContainer>> ftContainers)
    : MildRecommender(std::make_shared<FeatureAggregator>(ftContainers))
{
    for (const auto& ftContainer : ftContainers)
        mBlockList.push_back(ftContainer->getNumImgs());
}

std::vector<std::pair<std::size_t, std::size_t>> MildRecommender::getPairs(
    std::size_t, std::size_t window, const std::vector<std::size_t>& ids)
{
    // make orb feature container (sift does not work)
    /* std::cout << "Computing Features for MILD Recommender" << std::endl; */
    /* mFtContainer->compute(1000, ComputeBehavior::Keep, ids); */

    MILD::LoopClosureDetector lcd(FEATURE_TYPE_ORB, 16, 0);
    MILD::BayesianFilter filter(0.3, 4, 4, window);

    Eigen::VectorXf prevVisitProb(1);
    prevVisitProb << 0.1;
    std::vector<Eigen::VectorXf> prevVisitFlag;

    /* std::vector<std::pair<std::size_t, std::size_t>> pairs; */
    std::unordered_map<std::pair<std::size_t, std::size_t>, double> scores;

    auto transId = [&ids](std::size_t i) { return ids.empty() ? i : ids[i]; };

    std::size_t numImgs = ids.empty() ? mFtContainer->getNumImgs() : ids.size();
    std::cout << "Using MILD to get possible image pairs" << std::endl;
    ProgressBar bar(numImgs);
    for (std::size_t k = 0; k < numImgs; k++)
    {
        auto desc = mFtContainer->descriptorAt(transId(k));

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
        ++bar;
        bar.display();
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
    auto pairs = MatchesContainer::getKeyList(list);
    for (const auto [i, j] : pairs)
    {
        for (int n = -5; n <= 5; n++)
        {
            for (int m = -5; m <= 5; m++)
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


// is only called for inter video loop closure
void MildRecommender::filterPairList(std::vector<std::pair<std::size_t, std::size_t>>& pairs,
    std::size_t size) const
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
            if (pair.first - shift >=mBlockList[currBlock])
                shift += mBlockList[currBlock++];

            // delete intra video matches
            if (pair.second - shift >= mBlockList[currBlock])
                keepPairs.push_back(pair);
            else
                std::cout << pair.first << ", " << pair.second << std::endl;
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
