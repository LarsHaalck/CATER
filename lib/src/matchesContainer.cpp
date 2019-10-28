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

namespace ht
{
using Gt = GeometricType;
auto Put = Gt::Putative;
auto Hom = Gt::Homography;
auto Aff = Gt::Affinity;
auto Sim = Gt::Similarity;
auto Iso = Gt::Isometry;

MatchesContainer::MatchesContainer(std::shared_ptr<FeatureContainer> featureContainer,
    const fs::path& matchDir, MatchType matchType, std::size_t window, Gt geomType)
    : mFtContainer(std::move(featureContainer))
    , mMatchDir(matchDir)
    , mMatchType(matchType)
    , mWindow(window)
    , mGeomType(geomType | Put)
    , mIsComputed(true)
{
    // TODO: maybe smarter to only calculate missing, but this is fine for now
    if (static_cast<unsigned int>(mGeomType & Put))
    {
        auto matchFile = getFileName(detail::MatchTrafo::Match, Put);
        if (!fs::is_regular_file(matchFile))
            mIsComputed = false;
    }

    for (auto type : getTypeList())
    {
        if (!checkIfExists(type))
        {
            mIsComputed = false;
            break;
        }
    }

    if (matchType != MatchType::Manual && (!fs::exists(matchDir) || !fs::is_directory(matchDir)))
        fs::create_directories(matchDir);
}

bool MatchesContainer::checkIfExists(Gt geomType) const
{
    auto trafoFile = getFileName(detail::MatchTrafo::Trafo, geomType);
    auto matchFile = getFileName(detail::MatchTrafo::Match, geomType);
    if (!fs::is_regular_file(trafoFile) || !fs::is_regular_file(matchFile))
        return false;

    return true;
}

Gt MatchesContainer::getTypeFromFile(const fs::path& file) const
{
    auto type = file.stem().extension();
    if (type == ".put")
        return Put;
    if (type == ".h")
        return Hom;
    if (type == ".a")
        return Aff;
    if (type == ".s")
        return Sim;
    if (type == ".i")
        return Iso;

    throw UnknownGeometricType(type);
}

std::filesystem::path MatchesContainer::getFileName(
    detail::MatchTrafo matchTrafo, Gt geomType) const
{
    auto fullFile = mMatchDir;
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

std::vector<GeometricType> MatchesContainer::getTypeList() const
{
    std::vector<GeometricType> types;
    if (static_cast<unsigned int>(mGeomType & Hom))
        types.push_back(Hom);
    if (static_cast<unsigned int>(mGeomType & Aff))
        types.push_back(Aff);
    if (static_cast<unsigned int>(mGeomType & Sim))
        types.push_back(Sim);
    if (static_cast<unsigned int>(mGeomType & Iso))
        types.push_back(Iso);
    return types;
}

void MatchesContainer::compute(std::size_t cacheSize, ComputeBehavior behavior, const ImgIds& ids)
{
    if ((mIsComputed && behavior == ComputeBehavior::Keep) || mMatchType == MatchType::Manual)
    {
        std::cout << "Skipping Feature Matching..." << std::endl;
        return;
    }

    auto matches = getPutativeMatches(cacheSize, ids);
    for (auto type : getTypeList())
        matches = getGeomMatches(cacheSize, type, std::move(matches));
}

GeometricType MatchesContainer::getUsableTypes(const ImgIds& ids)
{
    GeometricType useableTypes = GeometricType::Undefined;
    for (auto type : getTypeList())
    {
        std::cout << "Checking usability of: " << type << std::endl;
        auto matches = loadMatches(type);
        auto numImgs = ids.empty() ? mFtContainer->getNumImgs() : ids.size();
        bool useable = true;
        for (std::size_t i = 1; i < numImgs; i++)
        {
            auto prevIdx = ids.empty() ? i - 1 : ids[i - 1];
            auto currIdx = ids.empty() ? i : ids[i];
            if (!matches.count(std::make_pair(prevIdx, currIdx)))
            {
                std::cout << "Missing link: " << prevIdx << " -> " << currIdx << std::endl;
                useable = false;
                break;
            }
        }
        if (useable)
            useableTypes |= type;
    }

    return useableTypes;
}

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
PairwiseMatches MatchesContainer::getGeomMatches(
    size_t cacheSize, Gt type, PairwiseMatches&& matches)
{
    auto filteredMatches = std::move(matches);
    auto pairList = getKeyList(filteredMatches);
    auto featCache = mFtContainer->getPairwiseFeatureCache(cacheSize, pairList);

    auto trafos = PairwiseTrafos();

    // TODO: is inplace modify via at() thread-safe?
    std::cout << "Computing geometric matches for " << typeToString(type) << "..." << std::endl;
    ProgressBar bar(featCache->getNumChunks());
    for (std::size_t i = 0; i < featCache->getNumChunks(); i++)
    {
        auto chunk = featCache->getChunk(i);
        auto [lower, _] = featCache->getChunkBounds(i);

#pragma omp parallel for
        for (std::size_t k = 0; k < featCache->getChunkSize(i); k++)
        {
            auto currPair = pairList[lower + k];
            auto featI = chunk[currPair.first];
            auto featJ = chunk[currPair.second];

            auto& currMatches = filteredMatches.at(currPair);

            auto currFilteredMatches = geomMatchPair(featI, featJ, type, currMatches);
            currMatches = currFilteredMatches.second;

            // maybe faster to insert placeholder transformation aka cv::Mat()
            // and update inplace without critical
            // needs truncating like filterEmptyMatches() afterwards
            if (!currMatches.empty())
            {
                #pragma omp critical
                trafos.insert(std::make_pair(currPair, currFilteredMatches.first));
            }
        }
        ++bar;
        bar.display();
    }

    filterEmptyMatches(filteredMatches);
    writeMatches(filteredMatches, type);
    writeTrafos(trafos, type);
    return filteredMatches;
}

void MatchesContainer::filterEmptyMatches(PairwiseMatches& matches)
{
    for (auto it = std::begin(matches); it != std::end(matches);)
    {
        if (it->second.empty())
            it = matches.erase(it);
        else
            ++it;
    }
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

cv::Ptr<cv::DescriptorMatcher> MatchesContainer::getMatcher() const
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

std::pair<Trafos, std::vector<Matches>> MatchesContainer::computePair(
    std::size_t idxI, std::size_t idxJ)
{
    auto descI = mFtContainer->descriptorAt(idxI);
    auto descJ = mFtContainer->descriptorAt(idxJ);
    auto descMatcher = getMatcher();

    Trafos trafos;
    std::vector<Matches> matches;
    auto currMatches = putMatchPair(descMatcher, descI, descJ);
    trafos.push_back(cv::Mat());
    matches.push_back(currMatches);

    auto featI = mFtContainer->featureAt(idxI);
    auto featJ = mFtContainer->featureAt(idxJ);
    for (auto type : getTypeList())
    {
        auto [trafo, geomMatches] = geomMatchPair(featI, featJ, type, currMatches);
        trafos.push_back(trafo);
        matches.push_back(geomMatches);

        currMatches.clear();
        currMatches = std::move(geomMatches);
    }

    return std::make_pair(trafos, matches);
}

std::pair<Trafo, Matches> MatchesContainer::geomMatchPair(const std::vector<cv::KeyPoint>& featI,
    const std::vector<cv::KeyPoint>& featJ, Gt filterType, const Matches& matches)
{
    std::vector<cv::Point2f> src, dst;
    for (size_t i = 0; i < matches.size(); i++)
    {
        src.push_back(featI[matches[i].queryIdx].pt);
        dst.push_back(featJ[matches[i].trainIdx].pt);
    }

    auto [mask, trafo] = getInlierMask(src, dst, filterType);
    std::vector<cv::DMatch> filteredMatches;
    for (size_t r = 0; r < mask.size(); r++)
    {
        if (mask[r])
            filteredMatches.push_back(matches[r]);
    }

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

PairwiseMatches MatchesContainer::getPutativeMatches(std::size_t cacheSize, const ImgIds& ids)
{
    auto pairList = getPairList(mFtContainer->getNumImgs(), ids);
    auto descCache = mFtContainer->getPairwiseDescriptorCache(cacheSize, pairList);
    auto descMatcher = getMatcher();

    std::cout << "Computing putative matches..." << std::endl;
    ProgressBar bar(descCache->getNumChunks());
    PairwiseMatches matches;
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

            auto currMatches = putMatchPair(descMatcher, descI, descJ);
            if (!currMatches.empty())
            {
#pragma omp critical
                matches.insert(std::make_pair(currPair, std::move(currMatches)));
            }
        }
        ++bar;
        bar.display();
    }
    writeMatches(matches, Put);
    return matches;
}

void MatchesContainer::writeMatches(const PairwiseMatches& matches, Gt type) const
{
    auto file = getFileName(detail::MatchTrafo::Match, type);
    std::ofstream stream(file.string(), std::ios::out | std::ios::binary);
    checkStream(stream, file);
    {
        cereal::PortableBinaryOutputArchive archive(stream);
        archive(matches);
    }
}

void MatchesContainer::writeTrafos(const PairwiseTrafos& trafos, Gt type) const
{
    auto file = getFileName(detail::MatchTrafo::Trafo, type);
    std::ofstream stream(file.string(), std::ios::out | std::ios::binary);
    checkStream(stream, file);
    {
        cereal::PortableBinaryOutputArchive archive(stream);
        archive(trafos);
    }
}

PairwiseMatches MatchesContainer::loadMatches(Gt type) const
{
    auto file = getFileName(detail::MatchTrafo::Match, type);
    std::ifstream stream(file.string(), std::ios::in | std::ios::binary);
    checkStream(stream, file);
    PairwiseMatches matches;
    {
        cereal::PortableBinaryInputArchive archive(stream);
        archive(matches);
    }
    return matches;
}

PairwiseTrafos MatchesContainer::loadTrafos(Gt type) const
{
    auto file = getFileName(detail::MatchTrafo::Trafo, type);
    std::ifstream stream(file.string(), std::ios::in | std::ios::binary);
    checkStream(stream, file);
    PairwiseTrafos trafos;
    {
        cereal::PortableBinaryInputArchive archive(stream);
        archive(trafos);
    }
    return trafos;
}

std::vector<std::pair<std::size_t, std::size_t>> MatchesContainer::getPairList(
    std::size_t size, const ImgIds& ids) const
{
    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    switch (mMatchType)
    {
    case MatchType::Exhaustive:
        pairs = getExhaustivePairList(size, ids);
        break;
    case MatchType::MILD:
        pairs = getMILDPairList(size, ids);
        break;
    case MatchType::Windowed:
        pairs = getWindowPairList(size, ids);
        break;
    default:
        pairs = {};
        break;
    }

    std::sort(std::begin(pairs), std::end(pairs));
    return pairs;
}

std::vector<std::pair<std::size_t, std::size_t>> MatchesContainer::getWindowPairList(
    std::size_t size, const ImgIds& ids) const
{
    if (!ids.empty())
        size = ids.size();

    std::vector<std::pair<std::size_t, std::size_t>> pairList;
    for (std::size_t i = 0; i < size; i++)
    {
        for (std::size_t j = i + 1; (j < i + mWindow) && (j < size); j++)
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
    std::size_t size, const ImgIds& ids) const
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

std::vector<std::pair<std::size_t, std::size_t>> MatchesContainer::getMILDPairList(
    std::size_t size, const ImgIds& ids) const
{
    // make orb feature container (sift does not work)
    auto ftContainer = std::make_shared<FeatureContainer>(
        mFtContainer->getImageContainer(), getMatchDir() / "MILD", FeatureType::ORB, 5000);

    ftContainer->compute(1000, ComputeBehavior::Keep, ids);

    MILD::LoopClosureDetector lcd(FEATURE_TYPE_ORB, 16, 0);
    MILD::BayesianFilter filter(0.3, 4, 4, mWindow);

    Eigen::VectorXf prevVisitProb(1);
    prevVisitProb << 0.1;
    std::vector<Eigen::VectorXf> prevVisitFlag;

    /* std::vector<std::pair<std::size_t, std::size_t>> pairs; */
    std::unordered_map<std::pair<std::size_t, std::size_t>, double> scores;

    auto transId = [&ids](std::size_t i) { return ids.empty() ? i : ids[i]; };

    std::size_t numImgs = ids.empty() ? ftContainer->getNumImgs() : ids.size();
    std::cout << "Using MILD to get possible image pairs" << std::endl;
    ProgressBar bar(numImgs);
    for (std::size_t k = 0; k < numImgs; k++)
    {
        auto desc = ftContainer->descriptorAt(transId(k));

        std::vector<float> simScore;
        simScore.clear();
        lcd.insert_and_query_database(desc, simScore);
        filter.filter(simScore, prevVisitProb, prevVisitFlag);

        if (prevVisitFlag.size() >= 1)
        {
            for (int i = 0; i < prevVisitFlag[prevVisitFlag.size() - 1].size(); i++)
            {
                scores[std::make_pair(i, k)] = prevVisitFlag[prevVisitFlag.size() - 1][i];
            }
        }
        ++bar;
        bar.display();
    }

    if (prevVisitFlag.size() >= 4)
    {
        for (int j = 0; j < 3; j++)
        {
            for (int i = 0; i < prevVisitFlag[prevVisitFlag.size() - (3 - j)].size(); i++)
            {
                scores[std::make_pair(i, numImgs - (3 - j))]
                    = prevVisitFlag[prevVisitFlag.size() - (3 - j)][i];
            }
        }
    }

    dilatePairList(scores, numImgs);
    auto windowPairs = getWindowPairList(size, ids);
    for (const auto& [pair, score] : scores)
    {
        if (score > 0)
            windowPairs.push_back(std::make_pair(transId(pair.first), transId(pair.second)));
    }

    return windowPairs;
}

void MatchesContainer::dilatePairList(
    std::unordered_map<std::pair<std::size_t, std::size_t>, double>& list, std::size_t size) const
{
    auto pairs = getKeyList(list);
    for (const auto [i, j] : pairs)
    {
        for (int n = -5; n <= 5; n++)
        {
            for (int m = -5; m <= 5; m++)
            {
                auto iShift = i + n;
                auto jShift = j + m;
                auto pairShift = std::make_pair(iShift, jShift);
                // check boundary conditions and if the key already exists, otherwise insert
                if (iShift < size && jShift < size && jShift > iShift + mWindow
                    && !list.count(pairShift))
                {
                    list.insert(std::make_pair(pairShift, 1.0));
                }
            }
        }
    }
}

fs::path MatchesContainer::getMatchDir() const { return mMatchDir; }

PairwiseMatches MatchesContainer::getMatches(GeometricType geomType)
{
    assert(static_cast<unsigned int>(geomType & mGeomType)
        && "Requested geometric type that was not computed");

    if (static_cast<unsigned int>(geomType & mGeomType))
        return loadMatches(geomType);

    return {};
}

PairwiseTrafos MatchesContainer::getTrafos(GeometricType geomType)
{
    assert(static_cast<unsigned int>(geomType & mGeomType)
        && "Requested geometric type that was not computed");

    if (static_cast<unsigned int>(geomType & mGeomType) && geomType != Put)
        return loadTrafos(geomType);

    return {};
}

} // namespace ht
