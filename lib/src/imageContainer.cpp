#include "habitrack/imageContainer.h"

#include <fstream>
#include <iostream>

#include <opencv2/imgproc.hpp>

namespace fs = std::filesystem;

namespace ht
{
ImageContainer::ImageContainer(
    const fs::path& path, ReadMode mode, cv::Vec3d weights, cv::Vec2d resize)
    : mMode(mode)
    , mWeights(weights)
    , mResize(resize)
{
    if (!fs::exists(path))
    {
        throw fs::filesystem_error("Image folder/file does not exist", path,
            std::make_error_code(std::errc::no_such_file_or_directory));
    }

    if (!fs::is_directory(path) && !fs::is_regular_file(path))
    {
        throw fs::filesystem_error("Image path is not a folder nor a file", path,
            std::make_error_code(std::errc::no_such_file_or_directory));
    }

    if (fs::is_directory(path))
        fillImageFilesFromFolder(path);
    else
        fillImageFilesFromFile(path);

    // sort files to cope with different operating system conventions
    std::sort(std::begin(mImageFiles), std::end(mImageFiles));

    auto firstImg = at(0);
    mImgSize = firstImg.size();
    if (resize[0] > 0 && resize[1] > 0)
    {
        mImgSize = cv::Size(
            std::round(resize[0] * mImgSize.width), std::round(resize[1] * mImgSize.height));
    }
}

void ImageContainer::fillImageFilesFromFolder(const fs::path& path)
{
    for (const auto& p : fs::recursive_directory_iterator(path))
    {
        if (p.is_regular_file())
        {
            auto extension = p.path().extension();
            if (extension == ".jpg" || extension == ".png" || extension == ".jpeg"
                || extension == ".tif" || extension == ".tiff" || extension == ".TIF"
                || extension == ".TIFF" || extension == ".JPG" || extension == ".PNG"
                || extension == ".JPEG")
            {
                mImageFiles.push_back(p.path());
            }
        }
    }
}

void ImageContainer::fillImageFilesFromFile(const fs::path& path)
{
    std::ifstream stream {path};
    std::string currLine;
    while (std::getline(stream, currLine))
    {
        if (!currLine.empty())
        {
            fs::path imgPath(currLine);
            if (!fs::is_regular_file(imgPath))
            {
                throw fs::filesystem_error("Image file from file list does not exist", imgPath,
                    std::make_error_code(std::errc::no_such_file_or_directory));
            }
            mImageFiles.push_back(imgPath);
        }
    }
}

ImageContainer::~ImageContainer() {}

cv::Mat ImageContainer::transformToWeightedGray(cv::Mat mat) const
{
    cv::Mat channels[3];
    cv::split(mat, channels);
    channels[0].convertTo(channels[0], CV_32FC1);
    channels[1].convertTo(channels[1], CV_32FC1);
    channels[2].convertTo(channels[2], CV_32FC1);

    channels[0] *= mWeights(0);
    channels[1] *= mWeights(1);
    channels[2] *= mWeights(2);

    channels[0] = channels[0] + channels[1] + channels[2];
    cv::Mat tmp;
    channels[0].convertTo(tmp, CV_8U);
    return tmp;
}

cv::Mat ImageContainer::at(ImgId idx) const
{
    assert(idx < size() && "idx out of range in ImageContainer::at()");

    cv::Mat mat;
    switch (mMode)
    {
        case ReadMode::Unchanged:
            mat = cv::imread(mImageFiles[idx], cv::ImreadModes::IMREAD_UNCHANGED);
            break;
        case ReadMode::SpecialGray:
        {
            mat = cv::imread(mImageFiles[idx], cv::ImreadModes::IMREAD_UNCHANGED);
            if (mat.channels() != 3)
                break;

            mat = transformToWeightedGray(mat);
            break;
        }
        case ReadMode::Gray:
            mat = cv::imread(mImageFiles[idx], cv::ImreadModes::IMREAD_GRAYSCALE);
            break;
        default:
            mat = cv::Mat();
    }

    if (mResize(0) > 0 && mResize(1) > 0)
    {
        cv::Mat resized;
        cv::resize(
            mat, resized, cv::Size(), mResize(0), mResize(1), cv::InterpolationFlags::INTER_CUBIC);
        return resized;
    }
    return mat;
}

std::size_t ImageContainer::size() const { return mImageFiles.size(); }

std::filesystem::path ImageContainer::getFileName(ImgId idx) const
{
    assert(idx < size() && "idx out of range in ImageContainer::getFileName()");
    auto file = mImageFiles[idx];
    auto path = fs::path(file);
    return path.filename();
}

ImageCache ImageContainer::getCache(std::size_t maxChunkSize, const ImgIds& ids) const
{
    auto numElems = size();
    return ImageCache{*this, numElems, maxChunkSize, ids};
}

cv::Size ImageContainer::getImgSize() const { return mImgSize; }
} // namespace ht
