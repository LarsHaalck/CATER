#include "habitrack/matchesContainer.h"
#include "habitrack/featureContainer.h"
#include "unknownGeometricType.h"
#include "progressBar.h"

#include "MILD/loop_closure_detector.h"

#include <fstream>
#include <iostream>

namespace fs = std::filesystem;

using g = ht::GeometricType;

namespace ht
{
MatchesContainer::MatchesContainer(std::shared_ptr<FeatureContainer> featureContainer,
    const fs::path& matchDir, MatchType matchType, std::size_t window,
    GeometricType geomType)
    : mFtContainer(std::move(featureContainer))
    , mMatchDir(matchDir)
    , mMatchType(matchType)
    , mWindow(window)
    , mGeomType(geomType)
    , mIsComputed(true)
{
    if (static_cast<unsigned int>(mGeomType & g::Homography))
    {
        if (!checkIfExists(g::Homography))
            mIsComputed = false;
    }
    if (static_cast<unsigned int>(mGeomType & g::Affinity))
    {
        if (!checkIfExists(g::Affinity))
            mIsComputed = false;
    }
    if (static_cast<unsigned int>(mGeomType & g::Similarity))
    {
        if (!checkIfExists(g::Similarity))
            mIsComputed = false;
    }
    if (static_cast<unsigned int>(mGeomType & g::Isometry))
    {
        if (!checkIfExists(g::Isometry))
            mIsComputed = false;
    }

    if (!fs::exists(matchDir) || !fs::is_directory(matchDir))
        fs::create_directories(matchDir);
}

bool MatchesContainer::checkIfExists(GeometricType geomType)
{
    auto trafoFile = getFileName(detail::MatchTrafo::Trafo, geomType);
    auto matchFile = getFileName(detail::MatchTrafo::Match, geomType);
    if (!fs::is_regular_file(trafoFile) || !fs::is_regular_file(matchFile))
        return false;

    return true;
}

GeometricType MatchesContainer::getTypeFromFile(const fs::path& file)
{
    auto type = file.stem().extension();
    if (type == ".put")
        return GeometricType::Putative;
    if (type == ".h")
        return GeometricType::Homography;
    if (type == ".a")
        return GeometricType::Affinity;
    if (type == ".s")
        return GeometricType::Similarity;
    if (type == ".i")
        return GeometricType::Isometry;

    throw UnknownGeometricType(type);
}

std::filesystem::path MatchesContainer::getFileName(detail::MatchTrafo matchTrafo,
    GeometricType geomType)
{
    auto fullFile = mMatchDir;
    if (matchTrafo == detail::MatchTrafo::Match)
        fullFile /= "matches";
    else
        fullFile /= "trafos";

    using g = GeometricType;
    switch (geomType)
    {
        case g::Putative:
            fullFile += ".put";
            break;
        case g::Homography:
            fullFile += ".h";
            break;
        case g::Affinity:
            fullFile += ".a";
            break;
        case g::Similarity:
            fullFile += ".s";
            break;
        case g::Isometry:
            fullFile += ".i";
            break;
        default:
            throw UnknownGeometricType();
    }
    fullFile += ".bin";
    return fullFile;
}

void MatchesContainer::compute(std::size_t cacheSize, ComputeBehavior behavior)
{
    if (mIsComputed && behavior == ComputeBehavior::Keep)
        return;

    /* auto ftPtr = getFtPtr(); */
    /* auto imgCache = mImgContainer->gray()->getCache(cacheSize, ImageType::Regular); */

    /* ProgressBar bar(imgCache->getNumChunks()); */
    /* for (std::size_t i = 0; i < imgCache->getNumChunks(); i++) */
    /* { */
    /*     auto chunk = imgCache->getChunk(i); */

    /*     std::vector<std::vector<cv::KeyPoint>> fts(imgCache->getChunkSize(i)); */
    /*     std::vector<cv::Mat> descs(imgCache->getChunkSize(i)); */
    /*     #pragma omp parallel for */
    /*     for (std::size_t j = 0; j < chunk.size(); j++) */
    /*         ftPtr->detectAndCompute(chunk[j], cv::Mat(), fts[j], descs[j]); */

    /*     ++bar; */
    /*     bar.display(); */
    /*     writeChunk(imgCache->getChunkBounds(i), fts, descs); */
    /* } */
    mIsComputed = true;
}

std::vector<std::pair<std::size_t, std::size_t>> MatchesContainer::getPairList(
    std::size_t size)
{
    switch (mMatchType)
    {
        case MatchType::Exhaustive:
            return getExhaustivePairList(size);
        case MatchType::MILD:
            return getMILDPairList(size);
        case MatchType::Windowed:
            return getWindowPairList(size);
        default:
            return {};

    }
}

std::vector<std::pair<std::size_t, std::size_t>> MatchesContainer::getWindowPairList(
    std::size_t size)
{
    std::vector<std::pair<std::size_t, std::size_t>> pairList;
    for (std::size_t i = 0; i < size; i++)
    {
        for (std::size_t j = i + 1; (j < i + mWindow) && (j < size); j++)
        {
            pairList.push_back(std::make_pair(i, j));
        }
    }
    return pairList;
}

std::vector<std::pair<std::size_t, std::size_t>> MatchesContainer::getMILDPairList(
    std::size_t size)
{
    // make orb feature container (sift does not work)
    /* auto ftContainer = std::make_shared<FeatureContainer>(...); */

    // insert descriptors into lcd
}

std::vector<std::pair<std::size_t, std::size_t>> MatchesContainer::getExhaustivePairList(
    std::size_t size)
{
    std::vector<std::pair<size_t, size_t>> pairList;
    for (size_t i = 0; i < size; i++)
    {
        for (size_t j = i + 1; j < size; j++)
        {
            pairList.push_back(std::make_pair(i, j));
        }
    }
    return pairList;
}

/* void FeatureContainer::writeChunk(std::pair<std::size_t, std::size_t> bounds, */
/*     const std::vector<std::vector<cv::KeyPoint>>& fts, const std::vector<cv::Mat>& descs) */
/* { */
/*     auto [lower, upper] = bounds; */

/*     for (std::size_t i = lower; i < upper; i++) */
/*     { */
/*         auto ftFileName = getFileName(i, detail::FtDesc::Feature); */
/*         auto descFileName = getFileName(i, detail::FtDesc::Descriptor); */
/*         writeFts(ftFileName, fts[i - lower]); */
/*         writeDescs(descFileName, descs[i - lower]); */
/*     } */
/* } */

/* void FeatureContainer::writeFts( */
/*     const fs::path& file, const std::vector<cv::KeyPoint>& fts) */
/* { */
/*     std::ofstream stream(file.string(), std::ios::out | std::ios::binary); */
/*     checkStream(stream, file); */

/*     { */
/*         cereal::PortableBinaryOutputArchive archive(stream); */
/*         archive(fts); */
/*     } */
/* } */

/* void FeatureContainer::writeDescs(const fs::path& file, const cv::Mat& descs) */
/* { */
/*     std::ofstream stream(file.string(), std::ios::out | std::ios::binary); */
/*     checkStream(stream, file); */

/*     { */
/*         cereal::PortableBinaryOutputArchive archive(stream); */
/*         archive(descs); */
/*     } */
/* } */

/* cv::Ptr<cv::Feature2D> FeatureContainer::getFtPtr() */
/* { */
/*     switch (mType) */
/*     { */
/*     case FeatureType::ORB: */
/*         return cv::ORB::create(mNumFeatures); */
/*     case FeatureType::SIFT: */
/*         return cv::xfeatures2d::SIFT::create(mNumFeatures); */
/*     default: */
/*         throw UnknownFeatureType(); */
/*     } */
/* } */
/* std::vector<cv::KeyPoint> FeatureContainer::featureAt( */
/*     std::size_t idx, ImageType imageType) */
/* { */
/*     assert(idx < mImgContainer->getNumImages(imageType) && mIsComputed */
/*         && "idx out of range in FeatureContainer::featureAt() or not computed"); */

/*     auto realIdx = mImgContainer->getImageIdx(idx, imageType); */
/*     auto file = getFileName(realIdx, detail::FtDesc::Feature); */
/*     std::ifstream stream(file.string(), std::ios::in | std::ios::binary); */
/*     checkStream(stream, file); */

/*     std::vector<cv::KeyPoint> fts; */
/*     { */
/*         cereal::PortableBinaryInputArchive archive(stream); */
/*         archive(fts); */
/*     } */
/*     return fts; */
/* } */

/* cv::Mat FeatureContainer::descriptorAt(std::size_t idx, ImageType imageType) */
/* { */
/*     assert(idx < mImgContainer->getNumImages(imageType) && mIsComputed */
/*         && "idx out of range in FeatureContainer::descriptorAt() or not computed"); */

/*     auto realIdx = mImgContainer->getImageIdx(idx, imageType); */
/*     auto file = getFileName(realIdx, detail::FtDesc::Descriptor); */
/*     std::ifstream stream(file.string(), std::ios::in | std::ios::binary); */
/*     checkStream(stream, file); */

/*     cv::Mat descs; */
/*     { */
/*         cereal::PortableBinaryInputArchive archive(stream); */
/*         archive(descs); */
/*     } */
/*     return descs; */
/* } */

/* std::unique_ptr<FeatureCache> FeatureContainer::getFeatureCache( */
/*     std::size_t maxChunkSize, ImageType imageType) */
/* { */
/*     auto numElems = mImgContainer->getNumImages(imageType); */
/*     return std::make_unique<FeatureCache>( */
/*         shared_from_this(), numElems, maxChunkSize, imageType); */
/* } */
/* std::unique_ptr<DescriptorCache> FeatureContainer::getDescriptorCache( */
/*     std::size_t maxChunkSize, ImageType imageType) */
/* { */
/*     auto numElems = mImgContainer->getNumImages(imageType); */
/*     return std::make_unique<DescriptorCache>( */
/*         shared_from_this(), numElems, maxChunkSize, imageType); */
/* } */
}
