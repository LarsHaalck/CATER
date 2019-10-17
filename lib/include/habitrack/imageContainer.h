#ifndef HABITRACK_IMAGE_CONTAINER_H
#define HABITRACK_IMAGE_CONTAINER_H

#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "habitrack/imageCache.h"


namespace ht
{
namespace detail
{
    struct ImageData
    {
        std::vector<std::string> mImageFiles; // holds filenames for all used images
        std::vector<std::size_t> mKeyFrames; // maps keyframes to original frames
    };

} // namespace detail

class ImageContainer : public std::enable_shared_from_this<ImageContainer>
{
public:
    ImageContainer(const std::filesystem::path& path);
    virtual ~ImageContainer();

    virtual cv::Mat at(std::size_t idx, bool isKeyFrame) const;
    std::size_t getNumImages() const;

    std::filesystem::path getFileName(std::size_t idx) const;

    template <typename T>
    void markAsKeyFrame(T&& keyIds)
    {
        mData->mKeyFrames = std::forward<std::vector<std::size_t>>(keyIds);
        std::sort(
            std::begin(mData->mKeyFrames), std::end(mData->mKeyFrames));

        assert(mData->mKeyFrames.front() >= 0
            && mData->mKeyFrames.back() < mData->mImageFiles.size()
            && "Key frames ids are out of range in markAsKeyFrame()");
    }
    std::vector<std::size_t> getKeyFrames() const;
    std::size_t getNumKeyFrames() const;
    std::size_t getKeyFrameIdx(std::size_t idx) const;

    std::unique_ptr<ImageCache> getCache(std::size_t maxChunkSize,
        bool useOnlyKeyFrames = false);

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
