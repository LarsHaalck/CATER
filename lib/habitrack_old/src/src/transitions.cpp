#include "habitrack/transitions.h"
#include "habitrack/idTranslator.h"
#include "mst.h"

namespace ht::transitions
{
std::unordered_map<std::pair<std::size_t, std::size_t>, std::pair<std::size_t, std::size_t>>
getMostProminantTransition(const PairwiseMatches& matches, const std::vector<std::size_t>& sizes)
{
    auto keys = MatchesContainer::getKeyList(matches);
    ht::Translator translator(sizes);

    std::unordered_map<std::pair<std::size_t, std::size_t>,
        std::tuple<std::size_t, std::size_t, std::size_t>>
        transitions;
    for (auto& pair : keys)
    {
        auto [idI, idJ] = pair;
        auto localIdI = translator.globalToLocal(idI);
        auto localIdJ = translator.globalToLocal(idJ);

        auto blockI = localIdI.first;
        auto blockJ = localIdJ.first;
        auto blockPair = std::make_pair(blockI, blockJ);

        if (!transitions.count(blockPair))
            transitions[blockPair] = std::make_tuple(idI, idJ, matches.at(pair).size());
        else
        {
            auto [oldI, oldJ, count] = transitions[blockPair];
            if (matches.at(pair).size() > count)
                transitions[blockPair] = std::make_tuple(idI, idJ, matches.at(pair).size());
        }
    }

    ht::Graph graph(sizes.size(), transitions.size());
    for (auto elem : transitions)
    {
        auto [blockI, blockJ] = elem.first;
        auto [idI, idJ, count] = elem.second;
        std::cout << "Transition from block: " << blockI << " --> " << blockJ << std::endl;
        std::cout << "With frames: " << idI << " --> " << idJ << ", count = " << count << std::endl;
        std::cout << "--------------------------------------------------" << std::endl;

        graph.addEdge(blockI, blockJ, -count);
    }

    auto edges = graph.kruskalMST();

    using Pair = std::pair<std::size_t, std::size_t>;
    std::unordered_map<Pair, Pair> optimalTransitions;

    for (auto blockPair : edges)
    {
        auto [idI, idJ, _] = transitions[blockPair];
        optimalTransitions.insert(std::make_pair(
            std::make_pair(blockPair.first, blockPair.second), std::make_pair(idI, idJ)));
    }

    for (auto elem : optimalTransitions)
    {
        auto [blockI, blockJ] = elem.first;
        auto [idI, idJ] = elem.second;
        std::cout << "Mapping " << blockI << " --> " << blockJ << " with " << idI << " --> " << idJ
                  << std::endl;
    }
    return optimalTransitions;
}
} // namespace ht
