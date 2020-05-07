#include "habitrack/matches.h"
#include "unknownFeatureType.h"

#include "matIO.h"
#include "matchesIO.h"
#include "progressBar.h"
#include "unknownGeometricType.h"

#include <fstream>
#include <iostream>
#include <unordered_map>

#include <opencv2/imgproc.hpp>

namespace fs = std::filesystem;

namespace ht
{
using Gt = GeometricType;
auto Put = Gt::Putative;
auto Hom = Gt::Homography;
auto Aff = Gt::Affinity;
auto Sim = Gt::Similarity;
auto Iso = Gt::Isometry;

bool MatchesContainer::isComputed(const fs::path& matchDir, Gt geomType)
{
    bool isComputed = true;

    // TODO: throw exception if no strategy passed but expected
    // TODO: throw when empty and not manual?
    // TODO: maybe smarter to only calculate missing, but this is fine for now
    if (static_cast<unsigned int>(geomType & Put))
    {
        auto matchFile = getFileName(matchDir, detail::MatchTrafo::Match, Put);
        if (!fs::is_regular_file(matchFile))
            isComputed = false;
    }

    for (auto type : getTypeList(geomType))
    {
        if (!checkIfExists(matchDir, type))
        {
            isComputed = false;
            break;
        }
    }

    return isComputed;
}

bool MatchesContainer::checkIfExists(const fs::path& matchDir, Gt geomType)
{
    auto trafoFile = getFileName(matchDir, detail::MatchTrafo::Trafo, geomType);
    auto matchFile = getFileName(matchDir, detail::MatchTrafo::Match, geomType);
    if (!fs::is_regular_file(trafoFile) || !fs::is_regular_file(matchFile))
        return false;

    return true;
}

std::filesystem::path MatchesContainer::getFileName(
    const fs::path& matchDir, detail::MatchTrafo matchTrafo, Gt geomType)
{
    auto fullFile = matchDir;
    if (matchTrafo == detail::MatchTrafo::Match)
        fullFile /= "matches";
    else
        fullFile /= "trafos";

    switch (geomType)
    {
    case Gt::Putative:
        fullFile += ".put";
        break;
    case Gt::Homography:
        fullFile += ".h";
        break;
    case Gt::Affinity:
        fullFile += ".a";
        break;
    case Gt::Similarity:
        fullFile += ".s";
        break;
    case Gt::Isometry:
        fullFile += ".i";
        break;
    default:
        throw UnknownGeometricType();
    }
    fullFile += ".bin";
    return fullFile;
}

std::vector<GeometricType> MatchesContainer::getTypeList(Gt type)
{
    std::vector<GeometricType> types;
    if (static_cast<unsigned int>(type & Hom))
        types.push_back(Hom);
    if (static_cast<unsigned int>(type & Aff))
        types.push_back(Aff);
    if (static_cast<unsigned int>(type & Sim))
        types.push_back(Sim);
    if (static_cast<unsigned int>(type & Iso))
        types.push_back(Iso);
    return types;
}
MatchesContainer MatchesContainer::compute(const fs::path& matchDir, GeometricType geomType,
    const BaseFeatureContainer& fts, MatchType matchType, std::size_t window, double minCoverage,
    std::unique_ptr<PairRecommender> recommender, std::size_t cacheSize, const size_t_vec& ids)
{
    if (matchType != MatchType::Manual && (!fs::exists(matchDir) || !fs::is_directory(matchDir)))
        fs::create_directories(matchDir);
    auto matches = getPutativeMatches(
        matchDir, fts, matchType, window, std::move(recommender), cacheSize, ids);
    for (auto type : getTypeList(geomType))
    {
        matches = getGeomMatches(matchDir, fts, type, minCoverage, fts.getImageSize().area(),
            cacheSize, std::move(matches));
    }
    return MatchesContainer();
}

PairwiseMatches MatchesContainer::getPutativeMatches(const fs::path& matchDir,
    const BaseFeatureContainer& fts, MatchType matchType, std::size_t window,
    std::unique_ptr<PairRecommender> recommender, std::size_t cacheSize, const size_t_vec& ids)
{
    auto pairList = getPairList(matchType, fts.size(), window, std::move(recommender), ids);
    auto descCache = fts.getPairwiseDescriptorCache(cacheSize, pairList);
    auto descMatcher = getMatcher(fts.getFeatureType());

    std::cout << "Computing putative matches..." << std::endl;
    ProgressBar bar(descCache.getNumChunks());
    PairwiseMatches matches;
    for (std::size_t i = 0; i < descCache.getNumChunks(); i++)
    {
        auto chunk = descCache.getChunk(i);
        auto lower = descCache.getChunkBounds(i).first;

        for (std::size_t k = 0; k < descCache.getChunkSize(i); k++)
        {
            auto currPair = pairList[lower + k];
            matches.insert(std::make_pair(currPair, Matches()));
        }

        #pragma omp parallel for
        for (std::size_t k = 0; k < descCache.getChunkSize(i); k++)
        {
            auto currPair = pairList[lower + k];
            auto descI = chunk[currPair.first];
            auto descJ = chunk[currPair.second];

            auto currMatches = putMatchPair(descMatcher, descI, descJ);
            if (!currMatches.empty())
            {
                /* #pragma omp critical */
                /* matches.insert(std::make_pair(currPair, std::move(currMatches))); */

                matches.at(currPair) = std::move(currMatches);
            }
        }
        ++bar;
        bar.display();
    }
    filterEmptyPairwise(matches);
    writeMatches(matchDir, matches, Put);
    return matches;
}

cv::Ptr<cv::DescriptorMatcher> MatchesContainer::getMatcher(FeatureType featureType)
{
    switch (featureType)
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

/* GeometricType MatchesContainer::getUsableTypes(const size_t_vec& ids) */
/* { */
/*     GeometricType useableTypes = GeometricType::Undefined; */
/*     for (auto type : getTypeList()) */
/*     { */
/*         std::cout << "Checking usability of: " << type << std::endl; */
/*         auto matches = loadMatches(type); */
/*         auto numImgs = ids.empty() ? mFtContainer->getNumImgs() : ids.size(); */
/*         bool useable = true; */
/*         for (std::size_t i = 1; i < numImgs; i++) */
/*         { */
/*             auto prevIdx = ids.empty() ? i - 1 : ids[i - 1]; */
/*             auto currIdx = ids.empty() ? i : ids[i]; */
/*             if (!matches.count(std::make_pair(prevIdx, currIdx))) */
/*             { */
/*                 std::cout << "Missing link: " << prevIdx << " -> " << currIdx << std::endl; */
/*                 useable = false; */
/*                 break; */
/*             } */
/*         } */
/*         if (useable) */
/*             useableTypes |= type; */
/*     } */

/*     return useableTypes; */
/* } */

std::string MatchesContainer::typeToString(Gt type)
{
    switch (type)
    {
    case Gt::Homography:
        return "homography";
    case Gt::Affinity:
        return "affinity";
    case Gt::Similarity:
        return "similarity";
    case Gt::Isometry:
        return "isometry";
    default:
        return "";
    }
}

PairwiseMatches MatchesContainer::getGeomMatches(const fs::path& matchDir,
    const BaseFeatureContainer& fts, Gt geomType, double minCoverage, int area,
    std::size_t cacheSize, PairwiseMatches&& matches)
{
    auto filteredMatches = std::move(matches);
    auto pairList = getKeyList(filteredMatches);
    auto featCache = fts.getPairwiseFeatureCache(cacheSize, pairList);

    auto trafos = PairwiseTrafos();

    // TODO: is inplace modify via at() thread-safe?
    // TODO: profile this and check if really faster
    std::cout << "Computing geometric matches for " << typeToString(geomType) << "..." << std::endl;
    ProgressBar bar(featCache.getNumChunks());
    for (std::size_t i = 0; i < featCache.getNumChunks(); i++)
    {
        auto chunk = featCache.getChunk(i);
        auto lower = featCache.getChunkBounds(i).first;

        for (std::size_t k = 0; k < featCache.getChunkSize(i); k++)
        {
            auto currPair = pairList[lower + k];
            trafos.insert(std::make_pair(currPair, cv::Mat()));
        }

#pragma omp parallel for
        for (std::size_t k = 0; k < featCache.getChunkSize(i); k++)
        {
            auto currPair = pairList[lower + k];
            auto featI = chunk[currPair.first];
            auto featJ = chunk[currPair.second];

            auto& currMatches = filteredMatches.at(currPair);

            auto currFilteredMatches
                = geomMatchPair(featI, featJ, geomType, minCoverage, area, currMatches);
            currMatches = currFilteredMatches.second;

            // maybe faster to insert placeholder transformation aka cv::Mat()
            // and update inplace without critical
            // needs truncating like filterEmptyMatches() afterwards
            if (!currMatches.empty())
            {
/* #pragma omp critical */
/*                 trafos.insert(std::make_pair(currPair, currFilteredMatches.first)); */
                trafos.at(currPair) = currFilteredMatches.first;
            }
        }
        ++bar;
        bar.display();
    }

    filterEmptyPairwise(trafos);
    filterEmptyPairwise(filteredMatches);
    writeMatches(matchDir, filteredMatches, geomType);
    writeTrafos(matchDir, trafos, geomType);
    return filteredMatches;
}

Matches MatchesContainer::putMatchPair(
    cv::Ptr<cv::DescriptorMatcher> descMatcher, const cv::Mat& descI, const cv::Mat& descJ)
{
    auto matchesI = putMatchPairHelper(descMatcher, descI, descJ);
    auto matchesJ = putMatchPairHelper(descMatcher, descJ, descI);

    std::vector<cv::DMatch> remainingMatches;
    for (const auto& matchI : matchesI)
    {
        for (const auto& matchJ : matchesJ)
        {
            if (matchI.trainIdx == matchJ.queryIdx && matchJ.trainIdx == matchI.queryIdx)
            {
                remainingMatches.push_back(matchI);
                break;
            }
        }
    }
    return remainingMatches;
}

Matches MatchesContainer::putMatchPairHelper(
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

/* std::pair<Trafos, std::vector<Matches>> MatchesContainer::computePair( */
/*     std::size_t idxI, std::size_t idxJ) const */
/* { */
/*     auto descI = mFtContainer->descriptorAt(idxI); */
/*     auto descJ = mFtContainer->descriptorAt(idxJ); */
/*     auto descMatcher = getMatcher(); */

/*     Trafos trafos; */
/*     std::vector<Matches> matches; */
/*     auto currMatches = putMatchPair(descMatcher, descI, descJ); */
/*     trafos.push_back(cv::Mat()); */
/*     matches.push_back(currMatches); */

/*     auto featI = mFtContainer->featureAt(idxI); */
/*     auto featJ = mFtContainer->featureAt(idxJ); */
/*     for (auto type : getTypeList()) */
/*     { */
/*         auto [trafo, geomMatches] = geomMatchPair(featI, featJ, type, currMatches); */
/*         trafos.push_back(trafo); */
/*         matches.push_back(geomMatches); */

/*         currMatches.clear(); */
/*         currMatches = std::move(geomMatches); */
/*     } */

/*     return std::make_pair(trafos, matches); */
/* } */

std::pair<Trafo, Matches> MatchesContainer::geomMatchPair(const std::vector<cv::KeyPoint>& featI,
    const std::vector<cv::KeyPoint>& featJ, Gt filterType, double minCoverage, int area,
    const Matches& matches)
{
    std::vector<cv::Point2f> src, dst;
    for (size_t i = 0; i < matches.size(); i++)
    {
        src.push_back(featI[matches[i].queryIdx].pt);
        dst.push_back(featJ[matches[i].trainIdx].pt);
    }

    auto [mask, trafo] = getInlierMask(src, dst, filterType);
    std::vector<cv::DMatch> filteredMatches;
    std::vector<cv::Point2f> srcFiltered, dstFiltered;
    for (size_t r = 0; r < mask.size(); r++)
    {
        if (mask[r])
        {
            filteredMatches.push_back(matches[r]);
            if (minCoverage)
            {
                srcFiltered.push_back(src[r]);
                dstFiltered.push_back(dst[r]);
            }
        }
    }

    if (minCoverage)
    {
        int rectI = cv::boundingRect(srcFiltered).area();
        int rectJ = cv::boundingRect(dstFiltered).area();
        if (rectI < minCoverage * area || rectJ < minCoverage * area)
            filteredMatches.clear();
    }

    if (filteredMatches.size() < 10)
        filteredMatches.clear();

    return std::make_pair(trafo, filteredMatches);
}

std::pair<std::vector<uchar>, cv::Mat> MatchesContainer::getInlierMask(
    const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst, Gt type)
{
    switch (type)
    {
    case Gt::Isometry:
        return getInlierMaskIsometry(src, dst);
    case Gt::Similarity:
        return getInlierMaskSimilarity(src, dst);
    case Gt::Affinity:
        return getInlierMaskAffinity(src, dst);
    case Gt::Homography:
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

void MatchesContainer::writeMatches(
    const fs::path& matchDir, const PairwiseMatches& matches, Gt type)
{
    auto file = getFileName(matchDir, detail::MatchTrafo::Match, type);
    std::ofstream stream(file.string(), std::ios::out | std::ios::binary);
    checkStream(stream, file);
    {
        cereal::PortableBinaryOutputArchive archive(stream);
        archive(matches);
    }
}

void MatchesContainer::writeTrafos(const fs::path& matchDir, const PairwiseTrafos& trafos, Gt type)
{
    auto file = getFileName(matchDir, detail::MatchTrafo::Trafo, type);
    std::ofstream stream(file.string(), std::ios::out | std::ios::binary);
    checkStream(stream, file);
    {
        cereal::PortableBinaryOutputArchive archive(stream);
        archive(trafos);
    }
}

/* PairwiseMatches MatchesContainer::loadMatches(Gt type) const */
/* { */
/*     auto file = getFileName(detail::MatchTrafo::Match, type); */
/*     std::ifstream stream(file.string(), std::ios::in | std::ios::binary); */
/*     checkStream(stream, file); */
/*     PairwiseMatches matches; */
/*     { */
/*         cereal::PortableBinaryInputArchive archive(stream); */
/*         archive(matches); */
/*     } */
/*     return matches; */
/* } */

/* PairwiseTrafos MatchesContainer::loadTrafos(Gt type) const */
/* { */
/*     auto file = getFileName(detail::MatchTrafo::Trafo, type); */
/*     std::ifstream stream(file.string(), std::ios::in | std::ios::binary); */
/*     checkStream(stream, file); */
/*     PairwiseTrafos trafos; */
/*     { */
/*         cereal::PortableBinaryInputArchive archive(stream); */
/*         archive(trafos); */
/*     } */
/*     return trafos; */
/* } */

std::vector<std::pair<std::size_t, std::size_t>> MatchesContainer::getPairList(MatchType type,
    std::size_t size, std::size_t window, std::unique_ptr<PairRecommender> recommender,
    const size_t_vec& ids)
{
    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    switch (type)
    {
    case MatchType::Exhaustive:
        pairs = getExhaustivePairList(size, ids);
        break;
    case MatchType::Windowed:
        pairs = getWindowPairList(size, window, ids);
        break;
    case MatchType::Strategy:
        pairs = recommender->getPairs(size, window, ids);
        break;
    default:
        pairs = {};
        break;
    }

    std::sort(std::begin(pairs), std::end(pairs));
    return pairs;
}

std::vector<std::pair<std::size_t, std::size_t>> MatchesContainer::getWindowPairList(
    std::size_t size, std::size_t window, const size_t_vec& ids)
{
    if (!ids.empty())
        size = ids.size();

    std::vector<std::pair<std::size_t, std::size_t>> pairList;
    for (std::size_t i = 0; i < size; i++)
    {
        for (std::size_t j = i + 1; (j < i + window) && (j < size); j++)
        {
            if (!ids.empty())
                pairList.push_back(std::make_pair(ids[i], ids[j]));
            else
                pairList.push_back(std::make_pair(i, j));
        }
    }

    return pairList;
}

std::vector<std::pair<std::size_t, std::size_t>> MatchesContainer::getExhaustivePairList(
    std::size_t size, const size_t_vec& ids)
{
    if (!ids.empty())
        size = ids.size();

    std::vector<std::pair<size_t, size_t>> pairList;
    for (size_t i = 0; i < size; i++)
    {
        for (size_t j = i + 1; j < size; j++)
        {
            if (!ids.empty())
                pairList.push_back(std::make_pair(ids[i], ids[j]));
            else
                pairList.push_back(std::make_pair(i, j));
        }
    }
    return pairList;
}

/* fs::path MatchesContainer::getMatchDir() const { return mMatchDir; } */

/* PairwiseMatches MatchesContainer::getMatches(GeometricType geomType) */
/* { */
/*     assert(static_cast<unsigned int>(geomType & mGeomType) */
/*         && "Requested geometric type that was not computed"); */

/*     if (static_cast<unsigned int>(geomType & mGeomType)) */
/*         return loadMatches(geomType); */

/*     return {}; */
/* } */

/* PairwiseTrafos MatchesContainer::getTrafos(GeometricType geomType) */
/* { */
/*     assert(static_cast<unsigned int>(geomType & mGeomType) */
/*         && "Requested geometric type that was not computed"); */

/*     if (static_cast<unsigned int>(geomType & mGeomType) && geomType != Put) */
/*         return loadTrafos(geomType); */

/*     return {}; */
/* } */

} // namespace ht
