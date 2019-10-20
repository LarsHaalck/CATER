#include "habitrack/featureContainer.h"
#include "habitrack/imageContainer.h"
#include "unknownFeatureType.h"
#include "progressBar.h"

#include <opencv2/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <fstream>
#include <iostream>

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
    // TODO: should I be able to instantiate the container on key frames only?
    for (std::size_t i = 0; i < mImgContainer->getNumImages(ImageType::Regular); i++)
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

std::filesystem::path FeatureContainer::getFileName(
    std::size_t idx, detail::FtDesc ftDesc)
{
    auto stem = mImgContainer->getFileName(idx).stem();
    auto fullFile = mFtDir / stem;
    fullFile += (ftDesc == detail::FtDesc::Feature) ? "-ft" : "-desc";
    fullFile += (mType == FeatureType::ORB) ? ".orb" : ".sift";
    fullFile += ".bin";
    return fullFile;
}

void FeatureContainer::compute(std::size_t cacheSize, ComputeBehavior behavior)
{
    if (mIsComputed && behavior == ComputeBehavior::Keep)
        return;

    auto ftPtr = getFtPtr();
    auto imgCache = mImgContainer->gray()->getCache(cacheSize, ImageType::Regular);

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
    mIsComputed = true;
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

void FeatureContainer::writeFts(
    const fs::path& file, const std::vector<cv::KeyPoint>& fts)
{
    std::ofstream stream(file.string(), std::ios::out | std::ios::binary);
    checkStream(stream, file);

    {
        cereal::PortableBinaryOutputArchive archive(stream);
        archive(fts);
    }
}

void FeatureContainer::writeDescs(const fs::path& file, const cv::Mat& descs)
{
    std::ofstream stream(file.string(), std::ios::out | std::ios::binary);
    checkStream(stream, file);

    {
        cereal::PortableBinaryOutputArchive archive(stream);
        archive(descs);
    }
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
std::vector<cv::KeyPoint> FeatureContainer::featureAt(
    std::size_t idx, ImageType imageType)
{
    assert(idx < mImgContainer->getNumImages(imageType) && mIsComputed
        && "idx out of range in FeatureContainer::featureAt() or not computed");

    auto realIdx = mImgContainer->getImageIdx(idx, imageType);
    auto file = getFileName(realIdx, detail::FtDesc::Feature);
    std::ifstream stream(file.string(), std::ios::in | std::ios::binary);
    checkStream(stream, file);

    std::vector<cv::KeyPoint> fts;
    {
        cereal::PortableBinaryInputArchive archive(stream);
        archive(fts);
    }
    return fts;
}

cv::Mat FeatureContainer::descriptorAt(std::size_t idx, ImageType imageType)
{
    assert(idx < mImgContainer->getNumImages(imageType) && mIsComputed
        && "idx out of range in FeatureContainer::descriptorAt() or not computed");

    auto realIdx = mImgContainer->getImageIdx(idx, imageType);
    auto file = getFileName(realIdx, detail::FtDesc::Descriptor);
    std::ifstream stream(file.string(), std::ios::in | std::ios::binary);
    checkStream(stream, file);

    cv::Mat descs;
    {
        cereal::PortableBinaryInputArchive archive(stream);
        archive(descs);
    }
    return descs;
}

std::unique_ptr<FeatureCache> FeatureContainer::getFeatureCache(
    std::size_t maxChunkSize, ImageType imageType)
{
    auto numElems = mImgContainer->getNumImages(imageType);
    return std::make_unique<FeatureCache>(
        shared_from_this(), numElems, maxChunkSize, imageType);
}
std::unique_ptr<DescriptorCache> FeatureContainer::getDescriptorCache(
    std::size_t maxChunkSize, ImageType imageType)
{
    auto numElems = mImgContainer->getNumImages(imageType);
    return std::make_unique<DescriptorCache>(
        shared_from_this(), numElems, maxChunkSize, imageType);
}
}
