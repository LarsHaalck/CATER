#ifndef CATER_TRANSITIONS_H
#define CATER_TRANSITIONS_H

#include <cater/image-processing/matches.h>
#include <cater/util/pairHash.h>

namespace ct
{
using Transitions = std::unordered_map<std::pair<std::size_t, std::size_t>,
    std::pair<std::size_t, std::size_t>, hash>;

Transitions getMostProminantTransition(
    const PairwiseMatches& matches, const std::vector<std::size_t>& sizes);
} // namespace ct
#endif // CATER_TRANSITIONS_H
