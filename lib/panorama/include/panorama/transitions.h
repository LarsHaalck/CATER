#ifndef HABITRACK_TRANSITIONS_H
#define HABITRACK_TRANSITIONS_H

#include "image-processing/matches.h"

namespace ht::transitions
{
std::unordered_map<std::pair<std::size_t, std::size_t>, std::pair<std::size_t, std::size_t>>
getMostProminantTransition(
    const matches::PairwiseMatches& matches, const std::vector<std::size_t>& sizes);
} // namespace ht
#endif // HABITRACK_TRANSITIONS_H
