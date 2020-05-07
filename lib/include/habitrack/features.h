#ifndef HABITRACK_FEATURES_H
#define HABITRACK_FEATURES_H

#include "habitrack/baseFeatureContainer.h"
#include "habitrack/descriptorCache.h"
#include "habitrack/featureCache.h"
#include "habitrack/images.h"
#include "habitrack/pairwiseDescriptorCache.h"
#include "habitrack/pairwiseFeatureCache.h"
#include <filesystem>
#include <opencv2/features2d.hpp>
#include <vector>

namespace ht
{
namespace detail
{
    enum class FtDesc : bool
    {
        Feature,
        Descriptor
    };
}

class Features : public BaseFeatureContainer
{
public:
    Features() = default;

    static Features compute(const Images& imgContainer, const std::filesystem::path& ftDir,
        FeatureType type, std::size_t numFeatures, std::size_t cacheSize = 0,
        const size_t_vec& ids = size_t_vec());

    static bool isComputed(const Images& imgContainer, const std::filesystem::path& ftDir,
        FeatureType type, const size_t_vec& ids = size_t_vec());

    static Features fromDir(const Images& imgContainer, const std::filesystem::path& ftDir,
        FeatureType type, const size_t_vec& ids = size_t_vec());

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

    cv::Size getImageSize() const override;
    FeatureType getFeatureType() const override;
    std::size_t size() const override;

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
        const std::filesystem::path& stem, detail::FtDesc ftDesc);
    static void writeFts(const std::filesystem::path& file, const std::vector<cv::KeyPoint>& fts);
    static void writeDescs(const std::filesystem::path& file, const cv::Mat& descs);

private:
    std::filesystem::path mFtDir;
    FeatureType mType;
    cv::Size mImgSize;
    std::unordered_map<std::size_t, std::filesystem::path> mFtStems;
};
} // namespace ht

#endif // HABITRACK_FEATURES_H
