#include <filesystem>
#include <iostream>
#include <memory>

#include <spdlog/spdlog.h>

#include "image-processing/features.h"
#include "image-processing/images.h"
#include "image-processing/matches.h"
#include "image-processing/mildRecommender.h"
#include "image-processing/superGlue.h"
#include "image-processing/util.h"
#include "panorama/keyFrameRecommender.h"
#include "panorama/keyFrames.h"
#include "panorama/panoramaStitcher.h"

#include "cxxopts.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace ht;
namespace fs = std::filesystem;

int main(int argc, const char** argv)
{
    /* spdlog::set_level(spdlog::level::debug); */

    fs::path basePath;
    bool showResults = false;
    bool force = false;
    int stage = 0;
    std::size_t cacheSize = 200;
    int cols = 2 * 1920;
    int rows = 2 * 1080;
    int numFts = 500;
    double minCoverage = 0.0;
    int matchInt = 0;
    int featureInt = 0;

    auto geomType = GeometricType::Similarity;

    cxxopts::Options options("full", "");
    options.add_options()("i,in", "base path containing imgs dir", cxxopts::value(basePath))(
        "v", "show results", cxxopts::value(showResults))("c", "cache", cxxopts::value(cacheSize))(
        "f", "force", cxxopts::value(force))("k, cols", "cols", cxxopts::value(cols))(
        "l, rows", "rows", cxxopts::value(rows))("n,num_fts", "num", cxxopts::value(numFts))(
        "m, min_coverage", "min coverage", cxxopts::value(minCoverage))(
        "g,feature", "feature type 0: ORB, 1: SIFT, 2: SuperPoint", cxxopts::value(featureInt))(
        "e,matching", "matching type 0: extensive, 1: MILD", cxxopts::value(matchInt))(
        "s,stage", "0: fts, 1: matches, 2: init, 3: global", cxxopts::value(stage));

    auto result = options.parse(argc, argv);
    if (result.count("in") != 1)
    {
        std::cout << options.help() << std::endl;
        return -1;
    }

    std::cout << "You called: " << std::endl;
    std::cout << "i: " << basePath.string() << std::endl;
    std::cout << "v: " << showResults << std::endl;
    std::cout << "c: " << cacheSize << std::endl;
    std::cout << "k: " << cols << std::endl;
    std::cout << "k: " << rows << std::endl;
    std::cout << "n: " << numFts << std::endl;
    std::cout << "m: " << minCoverage << std::endl;
    std::cout << "g: " << featureInt << std::endl;
    std::cout << "e: " << matchInt << std::endl;
    std::cout << "s: " << stage << std::endl;

    auto matchType
        = (matchInt == 0) ? matches::MatchType::Exhaustive : matches::MatchType::Strategy;

    auto ftType = FeatureType::ORB;
    if (featureInt == 1)
        ftType = FeatureType::SIFT;
    if (featureInt == 2)
        ftType = FeatureType::SuperPoint;

    // load images
    auto images = Images(basePath / "imgs");

    // calc features for all frames
    auto ftPath = basePath / "fts";
    auto features = Features();
    if (Features::isComputed(images, ftPath, ftType) && !force)
        features = Features::fromDir(images, ftPath, ftType);
    else
        features = Features::compute(images, ftPath, ftType, numFts, cacheSize);

    if (stage < 1)
        return 0;

    auto matchPath = basePath / "matches";
    if (!matches::isComputed(matchPath, geomType) || force)
    {
        if (matchType == ht::matches::MatchType::Strategy)
        {
            std::unique_ptr<PairRecommender> recommender;
            if (ftType == ht::FeatureType::ORB)
                recommender = std::make_unique<MildRecommender>(features, 1, true);
            else
            {
                auto featuresOrb = Features();
                if (Features::isComputed(images, ftPath, FeatureType::ORB) && !force)
                    featuresOrb = Features::fromDir(images, ftPath, FeatureType::ORB);
                else
                    featuresOrb
                        = Features::compute(images, ftPath, FeatureType::ORB, numFts, cacheSize);
                recommender = std::make_unique<MildRecommender>(featuresOrb, 1, true);
            }

            if (ftType == FeatureType::SuperPoint)
            {
                features = matches::SuperGlue("/data/arbeit/sg/indoor", 800)
                               .compute(images, ftPath, matchPath, geomType,
                                   matches::MatchType::Strategy, 50, 0.0, std::move(recommender),
                                   cacheSize);
            }
            else
            {
                matches::compute(matchPath, geomType, features, matches::MatchType::Strategy, 50,
                    0.0, std::move(recommender), cacheSize);
            }
        }
        else
        {
            if (ftType == FeatureType::SuperPoint)
            {
                features = matches::SuperGlue("/data/arbeit/sg/indoor", 800)
                               .compute(images, ftPath, matchPath, geomType,
                                   matches::MatchType::Exhaustive, 0, 0.0, nullptr,
                                   cacheSize);
            }
            else
            {
                matches::compute(matchPath, geomType, features, matches::MatchType::Exhaustive, 0,
                    0.0, nullptr, cacheSize);
            }
        }
    }

    if (stage < 2)
        return 0;

    // panoramas can only be calculated from theses Types
    GeometricType useableTypes = matches::getConnectedTypes(matchPath, geomType, images.size());
    auto typeList = typeToTypeList(useableTypes);
    std::cout << "Usable types for PanoramaStitcher:" << std::endl;
    for (auto type : typeList)
        std::cout << type << std::endl;

    auto keyFrames = getContinuousIds(0, images.size());
    auto geomPano = GeometricType::Similarity;
    auto stitcher = PanoramaStitcher(images, keyFrames, geomPano);

    // init trafos of keyframes by concatenating them
    stitcher.initTrafos(matches::getTrafos(matchPath, geomPano));
    if (showResults)
    {
        auto pano = std::get<0>(stitcher.stitchPano(cv::Size(cols, rows)));
        cv::imwrite((basePath / "pano0.png").string(), pano);
    }

    if (stage < 3)
        return 0;

    // globally optimized these keyframe tranformations and write them for later IVLC
    stitcher.globalOptimizeKeyFrames(features, matches::getMatches(matchPath, geomPano));
    stitcher.writeTrafos(basePath / "kfs/opt_trafos.bin");
    if (showResults)
    {
        auto pano = std::get<0>(stitcher.stitchPano(cv::Size(cols, rows)));
        cv::imwrite((basePath / "pano1.png").string(), pano);
    }

    stitcher.writeTrafos(basePath / "opt_trafos.yml", WriteType::Readable);
    return 0;
}
