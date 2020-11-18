#include "image-processing/features.h"

#include "featureIO.h"
#include "image-processing/images.h"
#include "image-processing/util.h"
#include "io/matIO.h"
#include "progressbar/progressBar.h"
#include "unknownFeatureType.h"
#include "util/stopWatch.h"
#include <fstream>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

using Stems = std::unordered_map<std::size_t, std::filesystem::path>;

namespace ht
{
Features::Features(
    const std::filesystem::path& ftDir, FeatureType type, cv::Size imageSize, const Stems& ftStems)
    : mFtDir(ftDir)
    , mType(type)
    , mImgSize(imageSize)
    , mFtStems(ftStems)
{
}

Features Features::compute(const Images& imgContainer, const std::filesystem::path& ftDir,
    FeatureType type, std::size_t numFeatures, std::size_t cacheSize, const size_t_vec& ids,
    std::shared_ptr<BaseProgressBar> cb)
{
    if (type == FeatureType::SuperPoint)
        return {};

    // make sure folder exits
    if (!fs::exists(ftDir) || !fs::is_directory(ftDir))
        fs::create_directories(ftDir);

    auto ftPtr = getFtPtr(type, numFeatures);
    auto imgCache = imgContainer.getCache(cacheSize, ids);

    spdlog::info("Computing Features");

    if (!cb)
        cb = std::make_shared<ProgressBar>();
    cb->status("Computing features");
    cb->setTotal(imgCache.getNumChunks());

    PreciseStopWatch timer;
    for (std::size_t i = 0; i < imgCache.getNumChunks(); i++)
    {
        auto chunk = imgCache.getChunk(i);
        spdlog::debug("Got chunk {}/{} with size {}", i + 1, imgCache.getNumChunks(), chunk.size());

        std::vector<std::vector<cv::KeyPoint>> fts(imgCache.getChunkSize(i));
        std::vector<cv::Mat> descs(imgCache.getChunkSize(i));
#pragma omp parallel for
        for (std::size_t j = 0; j < chunk.size(); j++)
        {
            ftPtr->detectAndCompute(chunk[j], cv::Mat(), fts[j], descs[j]);
            spdlog::debug("Features computed of image {} with size {}",
                std::get<0>(imgCache.getChunkBounds(i)) + j, fts[j].size());
        }

        cb->inc();
        writeChunk(imgContainer, ftDir, type, imgCache.getChunkBounds(i), fts, descs, ids);
    }

    auto elapsed_time = timer.elapsed_time<unsigned int, std::chrono::milliseconds>();
    cb->status("Finished");
    cb->done();
    spdlog::info("Computed Features in {} ms", elapsed_time);

    // sanity check
    if (isComputed(imgContainer, ftDir, type, ids))
        return Features::fromDir(imgContainer, ftDir, type, ids);

    spdlog::warn("Features were not computed sucessfully");
    return Features {ftDir, type, cv::Size(), Stems()};
}

Features Features::compute(const Images& imgContainer, const std::filesystem::path& ftDir,
    FeatureType type, std::size_t numFeatures, std::size_t start, std::size_t end,
    std::size_t cacheSize, std::shared_ptr<BaseProgressBar> cb)
{
    assert(end <= imgContainer.size()
        && "start must be > 0 and end must be smaller than number of images in "
           "Features::compute()");
    return compute(imgContainer, ftDir, type, numFeatures, cacheSize, getContinuousIds(start, end),
        std::move(cb));
}

bool Features::isComputed(const Images& imgContainer, const std::filesystem::path& ftDir,
    FeatureType type, const size_t_vec& ids)
{
    bool isComputed = true;

    if (ids.empty())
    {
        // check if some file is missing or other type expected
        for (std::size_t i = 0; i < imgContainer.size(); i++)
        {
            // only check for feature or descriptor because they are calculated together
            auto stem = imgContainer.getFileName(i).stem();
            auto ftFile = getFileName(ftDir, type, stem, FtDesc::Feature);
            if (!fs::is_regular_file(ftFile) || getTypeFromFile(ftFile) != type)
            {
                spdlog::debug("Missing feature file or wrong type: {}", ftFile.string());
                isComputed = false;
                break;
            }
        }
        return isComputed;
    }
    else
    {
        // check if some file is missing or other type expected
        for (auto i : ids)
        {
            // only check for feature or descriptor because they are calculated together
            auto stem = imgContainer.getFileName(i).stem();
            auto ftFile = getFileName(ftDir, type, stem, FtDesc::Feature);
            if (!fs::is_regular_file(ftFile) || getTypeFromFile(ftFile) != type)
            {
                spdlog::debug("Missing feature file or wrong type: {}", ftFile.string());
                isComputed = false;
                break;
            }
        }
        return isComputed;
    }
}

bool Features::isComputed(const Images& imgContainer, const std::filesystem::path& ftDir,
    FeatureType type, std::size_t start, std::size_t end)
{
    return isComputed(imgContainer, ftDir, type, getContinuousIds(start, end));
}

Features Features::fromDir(const Images& imgContainer, const std::filesystem::path& ftDir,
    FeatureType type, const size_t_vec& ids)
{
    spdlog::info("Reading Features from disk");
    Stems ftStems;
    if (ids.empty())
    {
        ftStems.reserve(imgContainer.size());
        for (std::size_t i = 0; i < imgContainer.size(); i++)
            ftStems.insert({i, imgContainer.getFileName(i).stem()});
    }
    else
    {
        ftStems.reserve(ids.size());
        for (auto i : ids)
            ftStems.insert({i, imgContainer.getFileName(i).stem()});
    }
    spdlog::debug("Found {} feature files available in folder {} for type {}", ftStems.size(),
        ftDir.string(), type);
    return Features {ftDir, type, imgContainer.getImgSize(), ftStems};
}
Features Features::fromDir(const Images& imgContainer, const fs::path& ftDir, FeatureType type,
    std::size_t start, std::size_t end)
{
    return fromDir(imgContainer, ftDir, type, getContinuousIds(start, end));
}

FeatureType Features::getTypeFromFile(const fs::path& file)
{
    auto type = file.stem().extension();
    if (type == ".orb")
        return FeatureType::ORB;
    if (type == ".sift")
        return FeatureType::SIFT;
    if (type == ".sp")
        return FeatureType::SuperPoint;

    throw UnknownFeatureType(type);
}

std::vector<cv::KeyPoint> Features::featureAt(std::size_t idx) const
{
    assert(mFtStems.count(idx) && "idx out of range in Features::featureAt()");

    auto stem = mFtStems.at(idx);
    auto file = getFileName(mFtDir, mType, stem, FtDesc::Feature);
    std::ifstream stream(file.string(), std::ios::in | std::ios::binary);
    io::checkStream(stream, file);
    std::vector<cv::KeyPoint> fts;
    {
        cereal::PortableBinaryInputArchive archive(stream);
        archive(fts);
    }
    return fts;
}

cv::Mat Features::descriptorAt(std::size_t idx) const
{
    assert(mFtStems.count(idx) && "idx out of range in Features::descriptorAt()");

    auto stem = mFtStems.at(idx);
    auto file = getFileName(mFtDir, mType, stem, FtDesc::Descriptor);
    std::ifstream stream(file.string(), std::ios::in | std::ios::binary);
    io::checkStream(stream, file);
    cv::Mat descs;
    {
        cereal::PortableBinaryInputArchive archive(stream);
        archive(descs);
    }
    return descs;
}

cv::Ptr<cv::Feature2D> Features::getFtPtr(FeatureType type, std::size_t numFeatures)
{
    switch (type)
    {
    case FeatureType::ORB:
        return cv::ORB::create(numFeatures);
    case FeatureType::SIFT:
        return cv::SIFT::create(numFeatures);
    default:
        throw UnknownFeatureType();
    }
}

void Features::writeChunk(const Images& imgContainer, const fs::path& ftDir, FeatureType type,
    std::pair<std::size_t, std::size_t> bounds, const std::vector<std::vector<cv::KeyPoint>>& fts,
    const std::vector<cv::Mat>& descs, const size_t_vec& ids)
{
    const auto [lower, upper] = bounds;
    for (std::size_t i = lower; i < upper; i++)
    {
        auto idx = ids.empty() ? i : ids[i];
        auto stem = imgContainer.getFileName(idx).stem();
        auto ftFileName = getFileName(ftDir, type, stem, FtDesc::Feature);
        auto descFileName = getFileName(ftDir, type, stem, FtDesc::Descriptor);
        writeFts(ftFileName, fts[i - lower]);
        writeDescs(descFileName, descs[i - lower]);
    }
}

std::filesystem::path Features::getFileName(
    const fs::path& ftDir, FeatureType type, const fs::path& stem, FtDesc ftDesc)
{
    auto fullFile = ftDir / stem;
    fullFile += (ftDesc == FtDesc::Feature) ? "-ft" : "-desc";

    if (type == FeatureType::ORB)
        fullFile += ".orb";
    else if (type == FeatureType::SIFT)
        fullFile += ".sift";
    else
        fullFile += ".sp";

    fullFile += ".bin";
    return fullFile;
}

void Features::writeFts(const fs::path& file, const std::vector<cv::KeyPoint>& fts)
{
    std::ofstream stream(file.string(), std::ios::out | std::ios::binary);
    io::checkStream(stream, file);
    {
        cereal::PortableBinaryOutputArchive archive(stream);
        archive(fts);
    }
}

void Features::writeDescs(const fs::path& file, const cv::Mat& descs)
{
    std::ofstream stream(file.string(), std::ios::out | std::ios::binary);
    io::checkStream(stream, file);
    {
        cereal::PortableBinaryOutputArchive archive(stream);
        archive(descs);
    }
}

FeatureCache Features::getFeatureCache(std::size_t maxChunkSize, const size_t_vec& ids) const
{
    assert(std::all_of(std::begin(ids), std::end(ids), [this](std::size_t i) {
        return this->mFtStems.count(i);
    }) && "Passed id list is not a subset of available Features in getFeatureCache()");

    return FeatureCache {*this, size(), maxChunkSize, ids};
}

DescriptorCache Features::getDescriptorCache(std::size_t maxChunkSize, const size_t_vec& ids) const
{
    assert(std::all_of(std::begin(ids), std::end(ids), [this](std::size_t i) {
        return this->mFtStems.count(i);
    }) && "Passed id list is not a subset of available Features in getDescriptorCache()");

    return DescriptorCache {*this, size(), maxChunkSize, ids};
}

PairwiseFeatureCache Features::getPairwiseFeatureCache(
    std::size_t maxChunkSize, const std::vector<std::pair<std::size_t, std::size_t>>& pairs) const
{
    assert(std::all_of(std::begin(pairs), std::end(pairs),
               [this](std::pair<std::size_t, std::size_t> pair) {
                   return (this->mFtStems.count(pair.first) && this->mFtStems.count(pair.second));
               })
        && "Passed id pair list is not a subset of available Features in "
           "getPairwiseFeatureCache()");

    return PairwiseFeatureCache(*this, maxChunkSize, pairs);
}

PairwiseDescriptorCache Features::getPairwiseDescriptorCache(
    std::size_t maxChunkSize, const std::vector<std::pair<std::size_t, std::size_t>>& pairs) const
{
    assert(std::all_of(std::begin(pairs), std::end(pairs),
               [this](std::pair<std::size_t, std::size_t> pair) {
                   return (this->mFtStems.count(pair.first) && this->mFtStems.count(pair.second));
               })
        && "Passed id pair list is not a subset of available Features in "
           "getPairwiseDescriptorCache()");

    return PairwiseDescriptorCache(*this, maxChunkSize, pairs);
}

} // namespace ht
