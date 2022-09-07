#include <cater/panorama/interExhaustiveRecommender.h>

#include <cater/image-processing/featureAggregator.h>
#include <cater/image-processing/idTranslator.h>

#include <spdlog/spdlog.h>

namespace ct
{
InterExhaustiveRecommender::InterExhaustiveRecommender(const FeatureAggregator& ftContainers)
    : mFtContainers(ftContainers)
    , mBlockList(ftContainers.getBlockList())
{
}

std::vector<std::pair<std::size_t, std::size_t>> InterExhaustiveRecommender::getPairs(
    std::size_t, std::size_t, const std::vector<std::size_t>& ids)
{
    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    auto transId = [&ids](std::size_t i) { return ids.empty() ? i : ids[i]; };
    std::size_t numImgs = ids.empty() ? mFtContainers.size() : ids.size();
    auto translator = Translator(mBlockList);
    for (std::size_t k = 0; k < numImgs; k++)
    {
        for (std::size_t l = k + 1; l < numImgs; l++)
        {
            auto k_trans = transId(k);
            auto l_trans = transId(l);
            if (translator.globalToLocal(k_trans).first != translator.globalToLocal(l_trans).first)
                pairs.push_back({k_trans, l_trans});
        }
    }
    return pairs;
}
} // namespace ct
