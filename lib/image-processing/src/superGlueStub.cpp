#include <habitrack/image-processing/superGlue.h>

namespace ht::matches
{

SuperGlue::SuperGlue(const std::filesystem::path&, int)
{
    throw std::logic_error("Not built with SuperGlue/Superpoint Support");
}

Features SuperGlue::compute(const Images&, const std::filesystem::path&,
    const std::filesystem::path&, GeometricType, MatchType, std::size_t, double,
    std::unique_ptr<PairRecommender>, std::size_t, const size_t_vec&,
    std::shared_ptr<BaseProgressBar>)
{
    return Features();
}

PairwiseMatches SuperGlue::putative(const Images&, const std::filesystem::path&,
    const std::filesystem::path&, MatchType, std::size_t, std::unique_ptr<PairRecommender>,
    const size_t_vec&, std::shared_ptr<BaseProgressBar>)
{
    return PairwiseMatches();
}

} // namespace ht
