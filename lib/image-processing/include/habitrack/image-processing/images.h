#ifndef HABITRACK_IMAGES_H
#define HABITRACK_IMAGES_H

#include <filesystem>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <habitrack/image-processing/baseImageContainer.h>

namespace ht
{
class Images final : public BaseImageContainer
{
public:
    enum class ReadMode
    {
        Gray,
        SpecialGray,
        Unchanged,
    };

    Images() = default; // needed for aggregator

    explicit Images(const std::vector<std::filesystem::path>& paths,
        ReadMode mode = ReadMode::Unchanged, cv::Vec3d weights = cv::Vec3d(),
        cv::Vec2d resize = cv::Vec2d(), cv::Rect2i crop = cv::Rect2i());

    explicit Images(const std::filesystem::path& path, ReadMode mode = ReadMode::Unchanged,
        cv::Vec3d weights = cv::Vec3d(), cv::Vec2d resize = cv::Vec2d(),
        cv::Rect2i crop = cv::Rect2i());

    std::size_t size() const override;
    ImageCache getCache(
        std::size_t maxChunkSize, const size_t_vec& ids = size_t_vec()) const override;
    PairwiseImageCache getPairwiseCache(std::size_t maxChunkSize,
        const std::vector<std::pair<std::size_t, std::size_t>>& pairs) const override;

    cv::Mat at(std::size_t idx) const override;
    cv::Size getImgSize() const override;
    cv::Point2f getCenter() const override;
    std::filesystem::path getFileName(std::size_t idx) const;

    void clip(std::size_t start, std::size_t end);

private:
    void fillImageFilesFromFolder(const std::filesystem::path& path);
    void fillImageFilesFromFile(const std::filesystem::path& path);
    void setImgSize();

    cv::Mat transformToWeightedGray(cv::Mat mat) const;

private:
    std::vector<std::filesystem::path> mImageFiles; // holds filenames for all images
    cv::Size mImgSize;
    ReadMode mMode;
    cv::Vec3d mWeights;
    cv::Vec2d mResize;
    cv::Rect2i mCrop;
};
} // namespace ht

#endif // HABITRACK_IMAGES_H
