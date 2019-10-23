#ifndef HABITRACK_IMAGE_CONTAINER_H
#define HABITRACK_IMAGE_CONTAINER_H

#include <filesystem>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "habitrack/imageCache.h"

namespace ht
{
using ImgId = std::size_t;
using ImgIds = std::vector<ImgId>;

namespace detail
{
    struct ImageData
    {
        std::vector<std::string> mImageFiles; // holds filenames for all images
    };

} // namespace detail

class ImageContainer : public std::enable_shared_from_this<ImageContainer>
{
public:
    ImageContainer(const std::filesystem::path& path);
    virtual ~ImageContainer();

    virtual cv::Mat at(ImgId idx) const;
    std::filesystem::path getFileName(ImgId idx) const;
    std::size_t getNumImgs() const;

    std::unique_ptr<ImageCache> getCache(std::size_t maxChunkSize,
        const ImgIds& ids = ImgIds());

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
