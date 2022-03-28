#include <habitrack/image-processing/images.h>

#include <fstream>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

namespace ht
{
Images::Images(
    const fs::path& path, ReadMode mode, cv::Vec3d weights, cv::Vec2d resize, cv::Rect2i crop)
    : mMode(mode)
    , mWeights(weights)
    , mResize(resize)
    , mCrop(crop)
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

    if (mode == ReadMode::SpecialGray && cv::countNonZero(weights) == 0)
        throw std::invalid_argument("Readmode SpecialGray cannot be used with all-zero-weights");
    if (mResize(0) * mResize(1) > 0 && mCrop.area() > 0)
        throw std::invalid_argument("Resize cannot be used with crop");

    if (fs::is_directory(path))
        fillImageFilesFromFolder(path);
    else
        fillImageFilesFromFile(path);

    if (mImageFiles.empty())
        spdlog::warn("No images were loaded");

    // sort files to cope with different operating system conventions
    std::sort(std::begin(mImageFiles), std::end(mImageFiles));
    setImgSize();
}

Images::Images(const std::vector<fs::path>& paths, ReadMode mode, cv::Vec3d weights,
    cv::Vec2d resize, cv::Rect2i crop)
    : mImageFiles(paths)
    , mMode(mode)
    , mWeights(weights)
    , mResize(resize)
    , mCrop(crop)
{
    for (auto p : paths)
    {
        if (!fs::exists(p) || !fs::is_regular_file(p))
        {
            throw fs::filesystem_error("Image file does not exist or is not a file", p,
                std::make_error_code(std::errc::no_such_file_or_directory));
        }
    }

    if (mode == ReadMode::SpecialGray && cv::countNonZero(weights) == 0)
        throw std::invalid_argument("Readmode SpecialGray cannot be used with all-zero-weights");
    if (mResize(0) * mResize(1) > 0 && mCrop.area() > 0)
        throw std::invalid_argument("Resize cannot be used with crop");

    setImgSize();
}

void Images::setImgSize()
{
    if (mImageFiles.empty())
        return;

    auto firstImg = at(0);
    mImgSize = firstImg.size();
    if (mResize(0) * mResize(1) > 0)
    {
        mImgSize = cv::Size(
            std::round(mResize(0) * mImgSize.width), std::round(mResize(1) * mImgSize.height));
    }
    if (mCrop.area() > 0)
        mImgSize = cv::Size(mCrop.width, mCrop.height);
}

void Images::fillImageFilesFromFolder(const fs::path& path)
{
    spdlog::info("Loading images from folder: {}", path.string());
    for (const auto& p : fs::directory_iterator(path))
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
            else
                spdlog::warn("Skipped file due to wrong extension: {}", p.path().string());
        }
    }
    spdlog::debug("Loaded {} images from folder: {}", mImageFiles.size(), path.string());
}

void Images::fillImageFilesFromFile(const fs::path& path)
{
    spdlog::info("Loading images from file: {}", path.string());
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
    spdlog::debug("Loaded {} images from file: {}", mImageFiles.size(), path.string());
}

cv::Mat Images::transformToWeightedGray(cv::Mat mat) const
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

cv::Mat Images::at(std::size_t idx) const
{
    assert(idx < size() && "idx out of range in Images::at()");

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

    if (mResize(0) * mResize(1) > 0)
    {
        cv::Mat resized;
        cv::resize(
            mat, resized, cv::Size(), mResize(0), mResize(1), cv::InterpolationFlags::INTER_CUBIC);
        return resized;
    }
    if (mCrop.area())
        return cv::Mat(mat, mCrop);
    return mat;
}

std::size_t Images::size() const { return mImageFiles.size(); }

fs::path Images::getFileName(std::size_t idx) const
{
    assert(idx < size() && "idx out of range in Images::getFileName()");
    auto file = mImageFiles[idx];
    auto path = fs::path(file);
    return path.filename();
}

void Images::clip(std::size_t start, std::size_t end)
{
    std::vector<fs::path> newImgFiles(
        std::begin(mImageFiles) + start, std::begin(mImageFiles) + end);
    mImageFiles.swap(newImgFiles);
}

ImageCache Images::getCache(std::size_t maxChunkSize, const size_t_vec& ids) const
{
    auto numElems = size();
    return ImageCache {*this, numElems, maxChunkSize, ids};
}

PairwiseImageCache Images::getPairwiseCache(
    std::size_t maxChunkSize, const std::vector<std::pair<std::size_t, std::size_t>>& pairs) const
{
    assert(std::all_of(std::begin(pairs), std::end(pairs),
               [this](std::pair<std::size_t, std::size_t> pair)
               { return (pair.first < this->size() && pair.second < this->size()); })
        && "Passed id pair list is not a subset of available Images in "
           "getPairwiseCache()");

    return PairwiseImageCache(*this, maxChunkSize, pairs);
}

cv::Size Images::getImgSize() const { return mImgSize; }
cv::Point2f Images::getCenter() const
{
    return cv::Point2f(mImgSize.width / 2.0f, mImgSize.height / 2.0f);
}
} // namespace ht
