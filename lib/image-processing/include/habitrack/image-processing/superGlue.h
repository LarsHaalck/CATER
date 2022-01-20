#ifndef HT_SUPERGLUE_H
#define HT_SUPERGLUE_H

#include <filesystem>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include <habitrack/image-processing/descriptorCache.h>
#include <habitrack/image-processing/featureCache.h>
#include <habitrack/image-processing/features.h>
#include <habitrack/image-processing/matches.h>
#include <habitrack/image-processing/pairwiseDescriptorCache.h>
#include <habitrack/image-processing/pairwiseFeatureCache.h>
#include <habitrack/progressbar/baseProgressBar.h>

namespace ht
{
class Images;
} // namespace ht

namespace ht::matches
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
} // namespace ht
#endif // HT_SUPERGLUE_H
