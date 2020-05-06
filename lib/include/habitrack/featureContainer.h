#ifndef HABITRACK_FEATURE_CONTAINER_H
#define HABITRACK_FEATURE_CONTAINER_H

#include "habitrack/baseFeatureContainer.h"
#include "habitrack/descriptorCache.h"
#include "habitrack/featureCache.h"
#include "habitrack/imageContainer.h"
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



class FeatureContainer : public BaseFeatureContainer
{
public:
    FeatureContainer() = default;

    static FeatureContainer compute(const ImageContainer& imgContainer,
        const std::filesystem::path& ftDir, FeatureType type, std::size_t numFeatures,
        std::size_t cacheSize = 0, const ImgIds& ids = ImgIds());

    static bool isComputed(const ImageContainer& imgContainer, const std::filesystem::path& ftDir,
        FeatureType type, const ImgIds& ids = ImgIds());

    static FeatureContainer fromDir(const ImageContainer& imgContainer,
        const std::filesystem::path& ftDir, FeatureType type, const ImgIds& ids = ImgIds());

    std::vector<cv::KeyPoint> featureAt(std::size_t idx) const override;
    cv::Mat descriptorAt(std::size_t idx) const override;

    FeatureCache getFeatureCache(
        std::size_t maxChunkSize, const ImgIds& ids = ImgIds()) const override;
    DescriptorCache getDescriptorCache(
        std::size_t maxChunkSize, const ImgIds& ids = ImgIds()) const override;
    PairwiseDescriptorCache getPairwiseDescriptorCache(std::size_t maxChunkSize,
        const std::vector<std::pair<std::size_t, std::size_t>>& pairs) const override;
    PairwiseFeatureCache getPairwiseFeatureCache(std::size_t maxChunkSize,
        const std::vector<std::pair<std::size_t, std::size_t>>& pairs) const override;

    FeatureType getFeatureType() const override;
    std::size_t size() const override;

private:
    FeatureContainer(const std::filesystem::path& ftDir, FeatureType type,
        const std::unordered_map<std::size_t, std::filesystem::path>& ftStems);

    static cv::Ptr<cv::Feature2D> getFtPtr(FeatureType type, std::size_t numFeatures);
    static FeatureType getTypeFromFile(const std::filesystem::path& file);

    static void writeChunk(const ImageContainer& imgContainer, const std::filesystem::path& ftDir,
        FeatureType type, std::pair<std::size_t, std::size_t> bounds,
        const std::vector<std::vector<cv::KeyPoint>>& fts, const std::vector<cv::Mat>& descs,
        const ImgIds& ids);

    static std::filesystem::path getFileName(const std::filesystem::path& ftDir, FeatureType type,
        const std::filesystem::path& stem, detail::FtDesc ftDesc);
    static void writeFts(const std::filesystem::path& file, const std::vector<cv::KeyPoint>& fts);
    static void writeDescs(const std::filesystem::path& file, const cv::Mat& descs);


private:
    std::filesystem::path mFtDir;
    FeatureType mType;
    std::unordered_map<std::size_t, std::filesystem::path> mFtStems;
};
} // namespace ht

#endif // HABITRACK_FEATURE_CONTAINER_H
