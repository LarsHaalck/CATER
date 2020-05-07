#ifndef HABITRACK_IMAGES_H
#define HABITRACK_IMAGES_H

#include <filesystem>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "habitrack/baseImageContainer.h"

namespace ht
{
enum class ReadMode
{
    Gray,
    SpecialGray,
    Unchanged,
};

class Images : public BaseImageContainer
{
public:
    Images() = default; // needed for aggregator

    Images(const std::filesystem::path& path,
        ReadMode mode = ReadMode::Unchanged,
        cv::Vec3d weights = cv::Vec3d(),
        cv::Vec2d resize = cv::Vec2d());

    std::size_t size() const;
    ImageCache getCache(std::size_t maxChunkSize, const size_t_vec& ids = size_t_vec()) const;

    cv::Mat at(std::size_t idx) const;
    cv::Size getImgSize() const;
    std::filesystem::path getFileName(std::size_t idx) const;

private:
    void fillImageFilesFromFolder(const std::filesystem::path& path);
    void fillImageFilesFromFile(const std::filesystem::path& path);

    cv::Mat transformToWeightedGray(cv::Mat mat) const;

private:
    std::vector<std::filesystem::path> mImageFiles; // holds filenames for all images
    cv::Size mImgSize;
    ReadMode mMode;
    cv::Vec3d mWeights;
    cv::Vec2d mResize;
};
} // namespace ht

#endif // HABITRACK_IMAGES_H
