#include "habitrack/imageContainer.h"

#include <fstream>
#include <iostream>

#include "habitrack/resizeDecorator.h"
#include "habitrack/grayDecorator.h"

namespace fs = std::filesystem;

namespace ht
{
ImageContainer::ImageContainer(const fs::path& path)
    : mData(std::make_shared<detail::ImageData>())
{
    if (!fs::exists(path))
    {
        throw fs::filesystem_error("Image folder/file does not exist",
            path, std::make_error_code(std::errc::no_such_file_or_directory));
    }

    if (!fs::is_directory(path) && !fs::is_regular_file(path))
    {
        throw fs::filesystem_error("Image path is not a folder nor a file",
            path, std::make_error_code(std::errc::no_such_file_or_directory));
    }

    if (fs::is_directory(path))
        fillImageFilesFromFolder(path);
    else
        fillImageFilesFromFile(path);

    std::sort(std::begin(mData->mImageFiles), std::end(mData->mImageFiles));
}

void ImageContainer::fillImageFilesFromFolder(const fs::path& path)
{
    for(const auto& p : fs::recursive_directory_iterator(path))
    {
        if (p.is_regular_file())
        {
            auto extension = p.path().extension();
            if (extension == ".jpg" || extension == ".png" || extension == ".jpeg"
                || extension == ".tif" || extension == ".tiff"
                || extension == ".TIF" || extension == ".TIFF"
                || extension == ".JPG" || extension == ".PNG" || extension == ".JPEG")
            {
                mData->mImageFiles.push_back(p.path().string());
            }
        }
    }

}

void ImageContainer::fillImageFilesFromFile(const fs::path& path)
{
    std::ifstream stream{path};
    std::string currLine;
    while(std::getline(stream, currLine))
    {
        if (!currLine.empty())
        {
            fs::path imgPath(currLine);
            if (!fs::is_regular_file(imgPath))
            {
                throw fs::filesystem_error("Image file from file list does not exist",
                    imgPath, std::make_error_code(std::errc::no_such_file_or_directory));
            }
            mData->mImageFiles.push_back(imgPath.string());
        }
    }
}

ImageContainer::~ImageContainer()
{
}

cv::Mat ImageContainer::at(std::size_t idx) const
{
    assert(idx <= mData->mImageFiles.size()
        && "idx out of range in ImageContainer::at()");
    return cv::imread(mData->mImageFiles[idx], cv::ImreadModes::IMREAD_UNCHANGED);
}

std::size_t ImageContainer::getNumImages() const { return mData->mImageFiles.size(); }
std::size_t ImageContainer::getNumKeyFrames() const { return mData->mKeyFrames.size(); }
std::vector<std::size_t> ImageContainer::getKeyFrames() const { return mData->mKeyFrames; }
std::shared_ptr<detail::ImageData> ImageContainer::getData() const { return mData; }

std::unique_ptr<ImageCache> ImageContainer::getCache(
    std::size_t maxChunkSize, bool useOnlyKeyFrames)
{
    return std::make_unique<ImageCache>(
        shared_from_this(), maxChunkSize, useOnlyKeyFrames);
}

std::shared_ptr<ImageContainer> ImageContainer::resize(double scale)
{
    return resize(scale, scale);
}
std::shared_ptr<ImageContainer> ImageContainer::resize(double scaleX, double scaleY)
{
    return std::make_shared<ResizeDecorator>(scaleX, scaleY, shared_from_this());
}
std::shared_ptr<ImageContainer> ImageContainer::gray()
{
    return std::make_shared<GrayDecorator>(shared_from_this());
}

ImageContainer::ImageContainer(std::shared_ptr<detail::ImageData> data)
    : mData(std::move(data))
{
}


} // namespace ht
