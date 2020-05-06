#ifndef HABITRACK_IMAGE_CONTAINER_H
#define HABITRACK_IMAGE_CONTAINER_H

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

class ImageContainer : public BaseImageContainer
{
public:
    ImageContainer(const std::filesystem::path& path,
        ReadMode mode = ReadMode::Unchanged,
        cv::Vec3d weights = cv::Vec3d(),
        cv::Vec2d resize = cv::Vec2d());
    virtual ~ImageContainer();

    std::size_t size() const;
    ImageCache getCache(std::size_t maxChunkSize, const ImgIds& ids = ImgIds()) const;

    virtual cv::Mat at(ImgId idx) const;
    virtual cv::Size getImgSize() const;
    std::filesystem::path getFileName(ImgId idx) const;

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

#endif // HABITRACK_IMAGE_CONTAINER_H
