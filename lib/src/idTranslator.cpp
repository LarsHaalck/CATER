#include "habitrack/idTranslator.h"

#include <numeric>

namespace ht::translate
{
std::vector<std::size_t> localToGlobal(
    const std::vector<std::vector<std::size_t>>& ids, const std::vector<std::size_t>& sizes)
{
    std::vector<std::size_t> globalIds;
    std::size_t cumSum = 0;
    for (std::size_t i = 0; i < ids.size(); i++)
    {
        for (auto id : ids[i])
            globalIds.push_back(id + cumSum);
        cumSum += sizes[i];
    }

    return globalIds;
}

} // namespace ht::translate

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
} // namespace ht
