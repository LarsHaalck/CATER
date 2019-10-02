#ifndef HABITRACK_IMAGE_CONTAINER_H
#define HABITRACK_IMAGE_CONTAINER_H

#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "habitrack/imageCache.h"


namespace ht
{
class ImageContainer : public std::enable_shared_from_this<ImageContainer>
{
public:
    ImageContainer(const std::filesystem::path& path);
    cv::Mat operator[](std::size_t idx) const;
    cv::Mat at(std::size_t idx) const;

    std::size_t getNumImages() const;
    std::size_t getNumKeyFrames() const;

    std::unique_ptr<ImageCache> getCache(
        std::size_t maxChunkSize, bool useOnlyKeyFrames = false);

    template <typename T>
    void markAsKeyFrame(T&& idx)
    {
        mKeyFrameMapping = std::forward<std::vector<std::size_t>>(idx);
        std::sort(std::begin(mKeyFrameMapping), std::end(mKeyFrameMapping));
    }

private:
    void fillImageFilesFromFolder(const std::filesystem::path& path);
    void fillImageFilesFromFile(const std::filesystem::path& path);

private:
    std::vector<std::string> mImageFiles; // holds filenames for all used images
    std::vector<std::size_t> mKeyFrameMapping; // maps keyframes to original frames
};
} // namespace ht

#endif // HABITRACK_IMAGE_CONTAINER_H
