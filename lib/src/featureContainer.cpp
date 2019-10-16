#include "habitrack/featureContainer.h"
#include "habitrack/imageContainer.h"
#include "habitrack/unknownFeatureType.h"
#include "progressBar.h"

#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <fstream>

namespace fs = std::filesystem;

namespace ht
{
FeatureContainer::FeatureContainer(std::shared_ptr<ImageContainer> imgContainer,
    const fs::path& ftDir, FeatureType type, std::size_t numFeatures)
    : mImgContainer(std::move(imgContainer))
    , mFtDir(ftDir)
    , mType(type)
    , mNumFeatures(numFeatures)
    , mIsComputed(true)
{
    // check if some file is missing or other type expected
    for (std::size_t i = 0; i < mImgContainer->getNumImages(); i++)
    {
        // only check for feature or descriptor because they are calculated together
        auto ftFile = getFileName(i, detail::FtDesc::Feature);
        if (!fs::is_regular_file(ftFile) || getTypeFromFile(ftFile) != mType)
        {
            mIsComputed = false;
            break;
        }
    }

    if (!fs::exists(ftDir) || !fs::is_directory(ftDir))
        fs::create_directories(ftDir);

}

FeatureType FeatureContainer::getTypeFromFile(const fs::path& file)
{
    auto type = file.stem().extension();

    if (type == ".orb")
        return FeatureType::ORB;
    if (type == ".sift")
        return FeatureType::SIFT;

    throw UnknownFeatureType(type);
}

std::filesystem::path FeatureContainer::getFileName(std::size_t idx,
    detail::FtDesc ftDesc)
{
    auto stem = mImgContainer->getFileName(idx).stem();
    auto fullFile = mFtDir / stem;
    fullFile += (ftDesc == detail::FtDesc::Feature) ? "-ft" : "-desc";
    fullFile += (mType == FeatureType::ORB) ? ".orb" : ".sift";
    fullFile += ".bin";
    return fullFile;
}

void FeatureContainer::compute(std::size_t cacheSize, bool overwrite)
{
    if (mIsComputed && !overwrite)
        return;

    auto ftPtr = getFtPtr();
    auto imgCache = mImgContainer->gray()->getCache(cacheSize);

    ProgressBar bar(imgCache->getNumChunks());
    for (std::size_t i = 0; i < imgCache->getNumChunks(); i++)
    {
        auto chunk = imgCache->getChunk(i);

        std::vector<std::vector<cv::KeyPoint>> fts(imgCache->getChunkSize(i));
        std::vector<cv::Mat> descs(imgCache->getChunkSize(i));
        #pragma omp parallel for
        for (std::size_t j = 0; j < chunk.size(); j++)
            ftPtr->detectAndCompute(chunk[j], cv::Mat(), fts[j], descs[j]);

        ++bar;
        bar.display();
        writeChunk(imgCache->getChunkBounds(i), fts, descs);
    }
}

void FeatureContainer::writeChunk(std::pair<std::size_t, std::size_t> bounds,
    const std::vector<std::vector<cv::KeyPoint>>& fts, const std::vector<cv::Mat>& descs)
{
    auto [lower, upper] = bounds;

    for (std::size_t i = lower; i < upper; i++)
    {
        auto ftFileName = getFileName(i, detail::FtDesc::Feature);
        auto descFileName = getFileName(i, detail::FtDesc::Descriptor);
        writeFts(ftFileName, fts[i - lower]);
        writeDescs(descFileName, descs[i - lower]);
    }
}

void FeatureContainer::writeFts(const fs::path& file, const std::vector<cv::KeyPoint>& fts)
{
    std::ofstream stream(file.string(), std::ios::out | std::ios::binary);
    if (!stream.is_open())
    {
        throw std::filesystem::filesystem_error("Error opening feature file",
                file, std::make_error_code(std::errc::io_error));
    }

    cereal::PortableBinaryOutputArchive archive(stream);
    archive(fts);
    stream.close();
}

void FeatureContainer::writeDescs(const fs::path& file, const cv::Mat& descs)
{
    std::ofstream stream(file.string(), std::ios::out | std::ios::binary);
    if (!stream.is_open())
    {
        throw std::filesystem::filesystem_error("Error opening descriptor file",
                file, std::make_error_code(std::errc::io_error));
    }

    cereal::PortableBinaryOutputArchive archive(stream);
    archive(descs);
    stream.close();
}

cv::Ptr<cv::Feature2D> FeatureContainer::getFtPtr()
{
    switch (mType)
    {
        case FeatureType::ORB:
            return cv::ORB::create(mNumFeatures);
        case FeatureType::SIFT:
            return cv::xfeatures2d::SIFT::create(mNumFeatures);
        default:
            throw UnknownFeatureType();
    }
}
}
