#include "habitrack/imageContainer.h"

#include <fstream>

#include <iostream>

namespace fs = std::filesystem;

namespace ht
{
ImageContainer::ImageContainer(const fs::path& path)
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

    std::sort(std::begin(mImageFiles), std::end(mImageFiles));
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
                mImageFiles.push_back(p.path().string());
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
            mImageFiles.push_back(imgPath.string());
        }
    }
}

cv::Mat ImageContainer::operator[](std::size_t idx) const
{
    return cv::Mat();
}

cv::Mat ImageContainer::at(std::size_t idx) const
{
    assert(idx > 0 && idx < mImageFiles.size()
        && "idx out of range in ImageContainer::at()");
    return operator[](idx);
}

std::size_t ImageContainer::getNumImages() const { return mImageFiles.size(); }
std::size_t ImageContainer::getNumKeyFrames() const { return mKeyFrameMapping.size(); }

std::unique_ptr<ImageCache> ImageContainer::getCache(
    std::size_t maxChunkSize, bool useOnlyKeyFrames)
{
    return std::make_unique<ImageCache>(
        shared_from_this(), maxChunkSize, useOnlyKeyFrames);
}

} // namespace ht
