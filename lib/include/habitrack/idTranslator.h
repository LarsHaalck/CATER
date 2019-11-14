#ifndef HABITRACK_ID_TRANSLATOR_H
#define HABITRACK_ID_TRANSLATOR_H

#include "habitrack/matchesContainer.h"
#include <vector>

namespace ht
{
class Translator
{
public:
    Translator(const std::vector<std::size_t>& sizes);

    std::pair<std::size_t, std::size_t> globalToLocal(std::size_t idx);

    std::size_t localToGlobal(std::pair<std::size_t, std::size_t> vidFrameIdx);
    std::vector<std::size_t> localToGlobal(const std::vector<std::vector<std::size_t>>& ids);
    PairwiseMatches localToGlobal(const std::vector<PairwiseMatches>& ids);
private:
    std::vector<std::size_t> mCumSums;
};
} // namespace ht
#endif // HABITRACK_ID_TRANSLATOR_H
