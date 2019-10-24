#include "habitrack/matchesContainer.h"
#include "habitrack/featureContainer.h"
#include "habitrack/imageContainer.h"
#include "unknownFeatureType.h"

#include "progressBar.h"
#include "unknownGeometricType.h"

#include "MILD/BayesianFilter.hpp"
#include "MILD/loop_closure_detector.h"

#include <fstream>
#include <iostream>
#include <unordered_map>

namespace fs = std::filesystem;

using g = ht::GeometricType;

namespace ht
{
/* constexpr std::pair<Trafo, std::vector<uchar>> zeroMaskTrafoTuple */

MatchesContainer::MatchesContainer(std::shared_ptr<FeatureContainer> featureContainer,
    const fs::path& matchDir, MatchType matchType, std::size_t window, GeometricType geomType)
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

std::filesystem::path MatchesContainer::getFileName(
    detail::MatchTrafo matchTrafo, GeometricType geomType)
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

    // maps pairs to their matches vector
    matches = getPutativeMatches(cacheSize, behavior);

    // TODO: is inplace modify via at() thread-safe?
    // chunk over it, modify in place

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
    /* mIsComputed = true; */
}

Matches MatchesContainer::putMatch(
    cv::Ptr<cv::DescriptorMatcher> descMatcher, const cv::Mat& descI, const cv::Mat& descJ)
{
    std::vector<cv::DMatch> currMatches;
    std::vector<std::vector<cv::DMatch>> knnMatches;
    descMatcher->knnMatch(descI, descJ, knnMatches, 2);

    for (std::size_t i = 0; i < knnMatches.size(); ++i)
    {
        if (knnMatches[i].size() < 2)
            continue;

        if (knnMatches[i][0].distance < 0.8 * knnMatches[i][1].distance)
            currMatches.push_back(knnMatches[i][0]);
    }

    return currMatches;
}

cv::Ptr<cv::DescriptorMatcher> MatchesContainer::getMatcher()
{
    switch (mFtContainer->getFtType())
    {
    case FeatureType::ORB:
        return cv::makePtr<cv::FlannBasedMatcher>(
            cv::makePtr<cv::flann::LshIndexParams>(20, 10, 2));
    case FeatureType::SIFT:
        return cv::DescriptorMatcher::create(cv::DescriptorMatcher::MatcherType::FLANNBASED);
    default:
        throw UnknownFeatureType("not orb or sift");
    }
}

std::pair<Trafos, std::vector<Matches>> MatchesContainer::compute(
    std::size_t idxI, std::size_t idxJ)
{
    auto descI = mFtContainer->descriptorAt(idxI);
    auto descJ = mFtContainer->descriptorAt(idxJ);
    auto descMatcher = getMatcher();

    Trafos trafos;
    std::vector<Matches> matches;
    auto currMatches = putMatch(descMatcher, descI, descJ);
    trafos.push_back(cv::Mat());
    matches.push_back(currMatches);

    auto hom = g::Homography;
    auto aff = g::Affinity;
    auto sim = g::Similarity;
    auto iso = g::Isometry;
    auto featI = mFtContainer->featureAt(idxI);
    auto featJ = mFtContainer->featureAt(idxJ);
    if (static_cast<unsigned int>(mGeomType & hom))
    {
        auto [trafo, geomMatches] = geomMatch(featI, featJ, hom, currMatches);
        trafos.push_back(trafo);
        matches.push_back(geomMatches);

        currMatches.clear();
        currMatches = std::move(geomMatches);
    }
    if (static_cast<unsigned int>(mGeomType & aff))
    {
        auto [trafo, geomMatches] = geomMatch(featI, featJ, aff, currMatches);
        matches.push_back(geomMatches);
        trafos.push_back(trafo);

        currMatches.clear();
        currMatches = std::move(geomMatches);
    }
    if (static_cast<unsigned int>(mGeomType & sim))
    {
        auto [trafo, geomMatches] = geomMatch(featI, featJ, sim, currMatches);
        matches.push_back(geomMatches);
        trafos.push_back(trafo);

        currMatches.clear();
        currMatches = std::move(geomMatches);
    }
    if (static_cast<unsigned int>(mGeomType & iso))
    {
        auto [trafo, geomMatches] = geomMatch(featI, featJ, iso, currMatches);
        matches.push_back(geomMatches);
        trafos.push_back(trafo);

        currMatches.clear();
        currMatches = std::move(geomMatches);
    }

    return std::make_pair(trafos, matches);
}

std::pair<Trafo, Matches> MatchesContainer::geomMatch(const std::vector<cv::KeyPoint>& featI,
    const std::vector<cv::KeyPoint>& featJ, GeometricType filterType, const Matches& matches)
{
    std::vector<cv::Point2f> src, dst;
    for (size_t i = 0; i < matches.size(); i++)
    {
        src.push_back(featI[matches[i].queryIdx].pt);
        dst.push_back(featJ[matches[i].trainIdx].pt);
    }

    auto [mask, trafo] = getInlierMask(src, dst, filterType);

    std::vector<cv::DMatch> filteredMatches;
    /* std::vector<cv::Point2f> srcFiltered, dstFiltered; */
    for (size_t r = 0; r < mask.size(); r++)
    {
        if (mask[r])
        {
            filteredMatches.push_back(matches[r]);
            /* srcFiltered.push_back(src[r]); */
            /* dstFiltered.push_back(dst[r]); */
        }
    }

    /* if (mMinCoverage) */
    /* { */
    /*     int rectI = cv::boundingRect(srcFiltered).area(); */
    /*     int rectJ = cv::boundingRect(dstFiltered).area(); */
    /*     int areaI = imgReader.getImage(idI).rows * imgReader.getImage(idI).cols; */
    /*     int areaJ = imgReader.getImage(idJ).rows * imgReader.getImage(idJ).cols; */
    /*     if (rectI < mMinCoverage * areaI || rectJ < mMinCoverage * areaJ) */
    /*         filteredMatches.clear(); */

    /* } */

    return std::make_pair(trafo, filteredMatches);
}

std::pair<std::vector<uchar>, cv::Mat> MatchesContainer::getInlierMask(
    const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst, g type)
{
    switch (type)
    {
    case g::Isometry:
        return getInlierMaskIsometry(src, dst);
    case g::Similarity:
        return getInlierMaskSimilarity(src, dst);
    case g::Affinity:
        return getInlierMaskAffinity(src, dst);
    case g::Homography:
        return getInlierMaskHomography(src, dst);
    default:
        return std::make_pair(std::vector<uchar>(), cv::Mat());
    }
    return std::make_pair(std::vector<uchar>(), cv::Mat());
}

std::pair<std::vector<uchar>, cv::Mat> MatchesContainer::getInlierMaskIsometry(
    const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst)
{
    cv::Mat mat;
    std::vector<uchar> mask;
    if (src.size() >= 2)
        mat = cv::estimateIsometry2D(src, dst, mask, cv::RANSAC);
    else
        return std::make_pair(std::vector<uchar>(), cv::Mat());

    if (getInlierCount(mask) < 2 * 2.5)
        return std::make_pair(std::vector<uchar>(), cv::Mat());

    return std::make_pair(mask, mat);
}

std::pair<std::vector<uchar>, cv::Mat> MatchesContainer::getInlierMaskSimilarity(
    const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst)
{
    cv::Mat mat;
    std::vector<uchar> mask;
    if (src.size() >= 2)
        mat = cv::estimateAffinePartial2D(src, dst, mask, cv::RANSAC);
    else
        return std::make_pair(std::vector<uchar>(), cv::Mat());

    if (getInlierCount(mask) < 2 * 2.5)
        return std::make_pair(std::vector<uchar>(), cv::Mat());

    return std::make_pair(mask, mat);
}

std::pair<std::vector<uchar>, cv::Mat> MatchesContainer::getInlierMaskAffinity(
    const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst)
{
    cv::Mat mat;
    std::vector<uchar> mask;
    if (src.size() >= 3)
        mat = cv::estimateAffine2D(src, dst, mask, cv::RANSAC);
    else
        return std::make_pair(std::vector<uchar>(), cv::Mat());

    if (getInlierCount(mask) < 3 * 2.5)
        return std::make_pair(std::vector<uchar>(), cv::Mat());

    return std::make_pair(mask, mat);
}

std::pair<std::vector<uchar>, cv::Mat> MatchesContainer::getInlierMaskHomography(
    const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst)
{
    std::vector<uchar> mask;
    cv::Mat mat;
    if (src.size() >= 4)
        mat = cv::findHomography(src, dst, mask, cv::RANSAC);
    else
        return std::make_pair(std::vector<uchar>(), cv::Mat());

    if (getInlierCount(mask) < 4 * 2.5)
        return std::make_pair(std::vector<uchar>(), cv::Mat());

    return std::make_pair(mask, mat);
}

PairWiseMatches MatchesContainer::getPutativeMatches(
    std::size_t cacheSize, ComputeBehavior behavior)
{
    if (mMatchType == MatchType::Manual)
        return;

    auto pairList = getPairList(cacheSize);
    std::sort(std::begin(pairList), std::end(pairList));
    auto descCache = mFtContainer->getPairwiseDescriptorCache(cacheSize, pairList);
    auto descMatcher = getMatcher();

    PairWiseMatches matches;
    for (std::size_t i = 0; i < descCache->getNumChunks(); i++)
    {
        auto chunk = descCache->getChunk(i);
        auto [lower, _] = descCache->getChunkBounds(i);

#pragma omp parallel for
        for (std::size_t k = 0; k < descCache->getChunkSize(i); k++)
        {
            auto currPair = pairList[lower + k];
            auto descI = chunk[currPair.first];
            auto descJ = chunk[currPair.second];

            auto currMatches = putMatch(descMatcher, descI, descJ);
            if (!currMatches.empty())
            {
#pragma omp critical
                matches.insert(std::make_pair(currPair, std::move(currMatches)));
            }
        }
    }
    writeMatches(matches, GeometricType::Putative);
}

void MatchesContainer::writeMatches(const PairWiseMatches& matches, GeometricType type) const
{
    std::ofstream stream("matches.put.bin", std::ios::out | std::ios::binary);
    checkStream(stream);
    {
        cereal::PortableBinaryOutputArchive archive(stream);
        archive(matches);
    }
}

std::vector<std::pair<std::size_t, std::size_t>> MatchesContainer::getPairList(std::size_t size)
{
    switch (mMatchType)
    {
    case MatchType::Exhaustive:
        return getExhaustivePairList(size);
    case MatchType::MILD:
        return {};
        /* return getMILDPairList(size); */
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

std::vector<std::pair<std::size_t, std::size_t>> MatchesContainer::getExhaustivePairList(
    std::size_t size)
{
    std::vector<std::pair<size_t, size_t>> pairList;
    for (size_t i = 0; i < size; i++)
    {
        for (size_t j = i + 1; j < size; j++)
            pairList.push_back(std::make_pair(i, j));
    }
    return pairList;
}

/* std::vector<std::pair<std::size_t, std::size_t>> MatchesContainer::getMILDPairList( */
/*     std::size_t size) */
/* { */
/*     std::cout << "Using MILD to get possible image pairs" << std::endl; */
/*     // make orb feature container (sift does not work) */
/*     auto ftContainer = std::make_shared<FeatureContainer>( */
/*         mFtContainer->getImageContainer(), mFtContainer->getFtDir(), FeatureType::ORB, */
/*         5000); */

/*     MILD::LoopClosureDetector lcd(FEATURE_TYPE_ORB, 16, 0); */
/*     MILD::BayesianFilter filter(0.6, 4, 4, 60); */
/*     ProgressBar bar(ftContainer->getNumImgs()); */

/*     Eigen::VectorXf prevVisitProb(1); */
/*     prevVisitProb << 0.1; */
/*     std::vector<Eigen::VectorXf> prevVisitFlag; */

/*     for (std::size_t i = 0; i < ftContainer->getNumImgs(); i++) */
/*     { */
/*         auto desc = ftContainer->descriptorAt(i); */
/*         std::vector<float> simScore; */
/*         lcd.insert_and_query_database(desc, simScore); */

/*         filter.filter(simScore, prevVisitProb, prevVisitFlag); */

/*         ++bar; */
/*         bar.display(); */
/*     } */

/*     // insert descriptors into lcd */

/*     return {}; */
/* } */
/* GeometricType MatchesContainer::findNextBestModel(GeometricType currType) */
/* { */
/*     using g = GeometricType; */
/*     switch (currType) */
/*     { */
/*         case g::Isometry: */
/*             if (static_cast<unsigned int>(mGeomType & g::Similarity)) */
/*                 return g::Similarity; */
/*             [[fallthrough]]; */
/*         case g::Similarity: */
/*             if (static_cast<unsigned int>(mGeomType & g::Affinity)) */
/*                 return g::Affinity; */
/*             [[fallthrough]]; */
/*         case g::Affinity: */
/*             if (static_cast<unsigned int>(mGeomType & g::Homography)) */
/*                 return g::Homography; */
/*             [[fallthrough]]; */
/*         case g::Homography: */
/*             return g::Putative; */
/*         default: */
/*             return g::Undefined; */
/*     } */
/* } */

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
