#include "panorama/transitions.h"
#include "mst.h"
#include "panorama/idTranslator.h"
#include <spdlog/spdlog.h>

namespace ht::transitions
{
std::unordered_map<std::pair<std::size_t, std::size_t>, std::pair<std::size_t, std::size_t>>
getMostProminantTransition(
    const PairwiseMatches& matches, const std::vector<std::size_t>& sizes)
{
    auto keys = matches::getKeyList(matches);
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

        //spdlog::debug("{}:{}, {}:{},c={}", blockI, blockJ, idI, idJ, matches.at(pair).size());
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
        spdlog::debug("Transition from block {} to {}", blockI, blockJ);
        spdlog::debug("With frames: {}, {}, count = {}", idI, idJ, count);

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
        spdlog::info("Mapping {} to {} with frame {} to {}", blockI, blockJ, idI, idJ);
    }
    return optimalTransitions;
}
} // namespace ht
