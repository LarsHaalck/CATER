#ifndef CT_SUPERGLUE_H
#define CT_SUPERGLUE_H

#include <filesystem>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include <cater/image-processing/descriptorCache.h>
#include <cater/image-processing/featureCache.h>
#include <cater/image-processing/features.h>
#include <cater/image-processing/matches.h>
#include <cater/image-processing/pairwiseDescriptorCache.h>
#include <cater/image-processing/pairwiseFeatureCache.h>
#include <cater/progressbar/baseProgressBar.h>

namespace ct
{
class Images;
} // namespace ct

namespace ct::matches
{
class SuperGlue
{
public:
    SuperGlue(const std::filesystem::path& zipFolder, int targetWidth);

    Features compute(const Images& imgContainer, const std::filesystem::path& ftDir,
        const std::filesystem::path& matchDir, GeometricType geomType, MatchType matchType,
        std::size_t window = 0, double minCoverage = 0.0,
        std::unique_ptr<PairRecommender> recommender = nullptr, std::size_t cacheSize = 0,
        const size_t_vec& ids = size_t_vec(), std::shared_ptr<BaseProgressBar> cb = {});

private:
    PairwiseMatches putative(const Images& imgContainer, const std::filesystem::path& ftDir,
        const std::filesystem::path& matchDir, MatchType matchType, std::size_t window,
        std::unique_ptr<PairRecommender> recommender, const size_t_vec& ids,
        std::shared_ptr<BaseProgressBar> cb);

private:
    std::filesystem::path mFolder;
    int mTargetWidth;
};
} // namespace ct
#endif // CT_SUPERGLUE_H
