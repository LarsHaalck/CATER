#include "habitrack/imageContainer.h"
#include "habitrack/resizeDecorator.h"
#include "habitrack/grayDecorator.h"

#include <fstream>
#include <iostream>

#include <opencv2/imgproc.hpp>


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

    // sort files to cope with different operating system conventions
    std::sort(std::begin(mData->mImageFiles), std::end(mData->mImageFiles));

    auto testImg = at(0);
    mData->mImgSize = testImg.size();
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

// TODO: imread type?
cv::Mat ImageContainer::at(ImgId idx) const
{
    assert(idx < getNumImgs() && "idx out of range in ImageContainer::at()");

    cv::Mat mat = cv::imread(mData->mImageFiles[idx], cv::ImreadModes::IMREAD_UNCHANGED);

    cv::Mat resMat;
    mat.convertTo(resMat, CV_8UC3);
    return resMat;
}

std::size_t ImageContainer::getNumImgs() const { return mData->mImageFiles.size(); }

std::filesystem::path ImageContainer::getFileName(ImgId idx) const
{
    auto file = mData->mImageFiles[idx];
    auto path = fs::path(file);
    return path.filename();
}

std::unique_ptr<ImageCache> ImageContainer::getCache(
    std::size_t maxChunkSize, const ImgIds& ids)
{
    auto numElems = getNumImgs();
    return std::make_unique<ImageCache>(
        shared_from_this(), numElems, maxChunkSize, ids);
}

cv::Size ImageContainer::getImgSize() const
{
    return mData->mImgSize;
}

std::shared_ptr<detail::ImageData> ImageContainer::getData() const { return mData; }
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
