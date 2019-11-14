#include "habitrack/idTranslator.h"

#include <numeric>

namespace ht
{
Translator::Translator(const std::vector<std::size_t>& sizes)
    : mCumSums(sizes.size())
{
    std::partial_sum(std::begin(sizes), std::end(sizes) - 1, std::begin(mCumSums) + 1);
}
std::pair<std::size_t, std::size_t> Translator::globalToLocal(std::size_t idx)
{
    auto pair = std::make_pair(0, 0);
    for (int i = static_cast<int>(mCumSums.size()) - 1; i >= 0; i--)
    {
        if (idx >= mCumSums[i])
        {
            pair.first = i;
            pair.second = idx - mCumSums[i];
            return pair;
        }
    }
    return pair;
}

std::size_t Translator::localToGlobal(std::pair<std::size_t, std::size_t> vidFrameId)
{
    return mCumSums[vidFrameId.first] + vidFrameId.second;
}

std::vector<std::size_t> Translator::localToGlobal(
    const std::vector<std::vector<std::size_t>>& ids)
{
    std::vector<std::size_t> globalIds;
    for (std::size_t i = 0; i < ids.size(); i++)
    {
        for (std::size_t j = 0; j < ids[i].size(); j++)
            globalIds.push_back(localToGlobal(std::make_pair(i, ids[i][j])));
    }
    return globalIds;
}

PairwiseMatches Translator::localToGlobal(const std::vector<PairwiseMatches>& matches)
{
    PairwiseMatches globalMatches;
    for (std::size_t i = 0; i < matches.size(); i++)
    {
        for (const auto& match : matches[i])
        {
            // get indexing pair and convert from current local block to global frame
            auto idxPair = match.first;
            idxPair.first = localToGlobal(std::make_pair(i, idxPair.first));
            idxPair.second = localToGlobal(std::make_pair(i, idxPair.second));

            // vector of cv::DMatch does not change
            globalMatches[idxPair] = match.second;
        }
    }
    return globalMatches;
}
} // namespace ht
