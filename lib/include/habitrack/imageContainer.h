#ifndef HABITRACK_IMAGE_CONTAINER_H
#define HABITRACK_IMAGE_CONTAINER_H

#include <filesystem>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "habitrack/baseImageContainer.h"

namespace ht
{
namespace detail
{
    struct ImageData
    {
        std::vector<std::string> mImageFiles; // holds filenames for all images
        cv::Size mImgSize;
    };

} // namespace detail

class ImageContainer : public BaseImageContainer,
                       public std::enable_shared_from_this<ImageContainer>
{
public:
    ImageContainer(const std::filesystem::path& path);
    virtual ~ImageContainer();

    std::size_t getNumImgs() const override;
    std::unique_ptr<ImageCache> getCache(
        std::size_t maxChunkSize, const ImgIds& ids = ImgIds()) override;

    // maybe overriden by decorator (e.g. resize)
    virtual cv::Mat at(ImgId idx) const override;
    virtual cv::Size getImgSize() const override;
    std::filesystem::path getFileName(ImgId idx) const;

    // decorator related methods
    std::shared_ptr<detail::ImageData> getData() const;
    std::shared_ptr<ImageContainer> resize(double scale);
    std::shared_ptr<ImageContainer> resize(double scaleX, double scaleY);
    std::shared_ptr<ImageContainer> gray();

private:
    void fillImageFilesFromFolder(const std::filesystem::path& path);
    void fillImageFilesFromFile(const std::filesystem::path& path);

protected:
    // only to be used by deriving decorator classes
    ImageContainer(std::shared_ptr<detail::ImageData> data);

private:
    std::shared_ptr<detail::ImageData> mData;
};
} // namespace ht

#endif // HABITRACK_IMAGE_CONTAINER_H
