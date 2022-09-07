#ifndef CATER_ID_TRANSLATOR_H
#define CATER_ID_TRANSLATOR_H

#include <cater/image-processing/matches.h>
#include <vector>

namespace ct
{
class Translator
{
public:
    Translator() = default;
    Translator(const std::vector<std::size_t>& sizes);

    std::pair<std::size_t, std::size_t> globalToLocal(std::size_t idx);

    std::size_t localToGlobal(std::pair<std::size_t, std::size_t> vidFrameIdx);
    std::vector<std::size_t> localToGlobal(const std::vector<std::vector<std::size_t>>& ids);
    PairwiseMatches localToGlobal(const std::vector<PairwiseMatches>& ids);

    // TODO: can this be comined with the vector version above with TMP?
    template <typename T>
    std::unordered_map<std::size_t, T> localToGlobal(
        const std::vector<std::unordered_map<std::size_t, T>>& ids)
    {
        std::unordered_map<std::size_t, T> globalIds;
        for (std::size_t i = 0; i < ids.size(); i++)
        {
            for(auto j : ids[i])
                globalIds.insert({localToGlobal(std::make_pair(i, j.first)), j.second});
        }
        return globalIds;
    }

private:
    std::vector<std::size_t> mCumSums;
};
} // namespace ct
#endif // CATER_ID_TRANSLATOR_H
