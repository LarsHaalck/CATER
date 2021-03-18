#ifndef HABITRACK_FEATURES_H
#define HABITRACK_FEATURES_H

#include <filesystem>
#include <opencv2/features2d.hpp>
#include <vector>

#include "image-processing/baseFeatureContainer.h"
#include "image-processing/descriptorCache.h"
#include "image-processing/featureCache.h"
#include "image-processing/pairwiseDescriptorCache.h"
#include "image-processing/pairwiseFeatureCache.h"
#include "progressbar/baseProgressBar.h"

namespace ht
{
class Images;
}

namespace ht::matches
{
class SuperGlue;
}

namespace ht
{
class Features final : public BaseFeatureContainer
{
private:
    enum class FtDesc : bool
    {
        Feature,
        Descriptor
    };

public:
    Features() = default;

    static Features compute(const Images& imgContainer, const std::filesystem::path& ftDir,
        FeatureType type, std::size_t numFeatures, std::size_t cacheSize = 0,
        const size_t_vec& ids = size_t_vec(), std::shared_ptr<BaseProgressBar> cb = {});
    static Features compute(const Images& imgContainer, const std::filesystem::path& ftDir,
        FeatureType type, std::size_t numFeatures, std::size_t start, std::size_t end,
        std::size_t cacheSize = 0, std::shared_ptr<BaseProgressBar> cb = {});

    static bool isComputed(const Images& imgContainer, const std::filesystem::path& ftDir,
        FeatureType type, const size_t_vec& ids = size_t_vec());
    static bool isComputed(const Images& imgContainer, const std::filesystem::path& ftDir,
        FeatureType type, std::size_t start, std::size_t end);

    static Features fromDir(const Images& imgContainer, const std::filesystem::path& ftDir,
        FeatureType type, const size_t_vec& ids = size_t_vec());
    static Features fromDir(const Images& imgContainer, const std::filesystem::path& ftDir,
        FeatureType type, std::size_t start, std::size_t end);

    std::vector<cv::KeyPoint> featureAt(std::size_t idx) const override;
    cv::Mat descriptorAt(std::size_t idx) const override;

    FeatureCache getFeatureCache(
        std::size_t maxChunkSize, const size_t_vec& ids = size_t_vec()) const override;
    DescriptorCache getDescriptorCache(
        std::size_t maxChunkSize, const size_t_vec& ids = size_t_vec()) const override;
    PairwiseDescriptorCache getPairwiseDescriptorCache(std::size_t maxChunkSize,
        const std::vector<std::pair<std::size_t, std::size_t>>& pairs) const override;
    PairwiseFeatureCache getPairwiseFeatureCache(std::size_t maxChunkSize,
        const std::vector<std::pair<std::size_t, std::size_t>>& pairs) const override;

    FeatureType getFeatureType() const override { return mType; }
    cv::Size getImageSize() const override { return mImgSize; }
    std::size_t size() const override { return mFtStems.size(); }

private:
    Features(const std::filesystem::path& ftDir, FeatureType type, cv::Size imageSize,
        const std::unordered_map<std::size_t, std::filesystem::path>& ftStems);

    static cv::Ptr<cv::Feature2D> getFtPtr(FeatureType type, std::size_t numFeatures);
    static FeatureType getTypeFromFile(const std::filesystem::path& file);

    static void writeChunk(const Images& imgContainer, const std::filesystem::path& ftDir,
        FeatureType type, std::pair<std::size_t, std::size_t> bounds,
        const std::vector<std::vector<cv::KeyPoint>>& fts, const std::vector<cv::Mat>& descs,
        const size_t_vec& ids);

    static std::filesystem::path getFileName(const std::filesystem::path& ftDir, FeatureType type,
        const std::filesystem::path& stem, FtDesc ftDesc);
    static void writeFts(const std::filesystem::path& file, const std::vector<cv::KeyPoint>& fts);
    static void writeDescs(const std::filesystem::path& file, const cv::Mat& descs);

private:
    std::filesystem::path mFtDir;
    FeatureType mType;
    cv::Size mImgSize;
    std::unordered_map<std::size_t, std::filesystem::path> mFtStems;

    // TODO rewrite this hacky bit
    friend class ht::matches::SuperGlue;
};

namespace fts::detail
{
}
} // namespace ht

#endif // HABITRACK_FEATURES_H
