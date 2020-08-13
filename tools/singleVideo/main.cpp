#include <filesystem>
#include <iostream>
#include <memory>

#include <spdlog/spdlog.h>

#include "image-processing/features.h"
#include "image-processing/images.h"
#include "image-processing/matches.h"
#include "image-processing/mildRecommender.h"
#include "panorama/keyFrameRecommender.h"
#include "panorama/keyFrames.h"
#include "panorama/panoramaStitcher.h"

#include "cxxopts.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace ht;
namespace fs = std::filesystem;
int main(int argc, char** argv)
{
    /* spdlog::set_level(spdlog::level::debug); */

    // TODO: add sift orb, sp argument
    // add option for exhaustive or mild
    fs::path basePath;
    bool showResults = false;
    bool force = false;
    int stage = 0;
    std::size_t cacheSize = 200;
    int cols = 2 * 1920;
    int rows = 2 * 1080;
    int numFts = 500;
    double minCoverage = 0.0;

    /* auto geomType = GeometricType::Homography | GeometricType::Affinity | GeometricType::Similarity */
    /*     | GeometricType::Isometry; */
    auto geomType = GeometricType::Similarity;

    cxxopts::Options options("single", "");
    options.add_options()("i,in", "base path containing imgs dir", cxxopts::value(basePath))(
        "v", "show results", cxxopts::value(showResults))("c", "cache", cxxopts::value(cacheSize))(
        "f", "force", cxxopts::value(force))("k, cols", "cols", cxxopts::value(cols))(
        "l, rows", "rows", cxxopts::value(rows))("n,num_fts", "num", cxxopts::value(numFts))(
        "m, min_coverage", "min coverage", cxxopts::value(minCoverage))("s,stage",
        "0: fts, 1: kfs: 2: matches, 3: init, 4: global, 5: reint; 6: global",
        cxxopts::value(stage));

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
    std::cout << "s: " << stage << std::endl;

    // load images
    auto images = Images(basePath / "imgs");

    // calc features for all frames
    auto ftPath = basePath / "fts";
    auto ftType = FeatureType::ORB;
    auto features = Features();
    if (Features::isComputed(images, ftPath, ftType) && !force)
        features = Features::fromDir(images, ftPath, ftType);
    else
        features = Features::compute(images, ftPath, ftType, numFts, cacheSize);

    if (stage < 1)
        return 0;

    // select key frames
    auto kfPath = basePath / "key_frames.yml";
    std::vector<std::size_t> keyFrames;
    if (KeyFrames::isComputed(kfPath) && !force)
        keyFrames = KeyFrames::fromDir(kfPath);
    else
        keyFrames = KeyFrames::compute(features, GeometricType::Similarity, kfPath, 0.3, 0.5);

    if (stage < 2)
        return 0;

    // calculate matches between images between keyframes via KeyFrameRecommender
    auto kfIntraPath = basePath / "kfs/maches_intra";
    auto kfRecommender = std::make_unique<KeyFrameRecommender>(keyFrames);
    if (!matches::isComputed(kfIntraPath, geomType) || force)
    {
        matches::compute(kfIntraPath, geomType, features, matches::MatchType::Strategy, 0, 0.0,
            std::move(kfRecommender), cacheSize);
    }

    // calculate matches between keyframes via exhaustive matching
    auto kfInterPath = basePath / "kfs/maches_inter";

    auto featuresDensePath = basePath / "kfs/fts";
    auto featuresDense = Features();
    if (Features::isComputed(images, featuresDensePath, ftType, keyFrames) && !force)
        featuresDense = Features::fromDir(images, featuresDensePath, ftType, keyFrames);
    else
    {
        featuresDense = Features::compute(
            images, featuresDensePath, ftType, 4 * numFts, cacheSize, keyFrames);
    }

    auto mildRecommender = std::make_unique<MildRecommender>(featuresDense, 1, true);
    if (!matches::isComputed(kfInterPath, geomType) || force)
    {
        // 4 because ceil(1 / 0.3) seems like a sensible default
        matches::compute(kfInterPath, geomType, featuresDense, matches::MatchType::Strategy, 4, 0.0,
            std::move(mildRecommender), cacheSize, keyFrames);
    }

    // panoramas can only be calculated from theses Types
    GeometricType useableTypes = matches::getConnectedTypes(kfInterPath, geomType, keyFrames);
    auto typeList = typeToTypeList(useableTypes);
    std::cout << "Usable types for PanoramaStitcher:" << std::endl;
    for (auto type : typeList)
        std::cout << type << std::endl;

    if (stage < 3)
        return 0;

    auto geomPano = GeometricType::Similarity;
    auto stitcher = PanoramaStitcher(images, keyFrames, geomPano);

    // init trafos of keyframes by concatenating them
    stitcher.initTrafos(matches::getTrafos(kfInterPath, geomPano));
    if (showResults)
    {
        auto pano = std::get<0>(stitcher.stitchPano(cv::Size(cols, rows)));
        cv::imwrite((basePath / "pano0.png").string(), pano);
    }

    if (stage < 4)
        return 0;

    // globally optimized these keyframe tranformations and write them for later IVLC
    stitcher.globalOptimizeKeyFrames(featuresDense, matches::getMatches(kfInterPath, geomPano));
    stitcher.writeTrafos(basePath / "kfs/opt_trafos.bin");
    if (showResults)
    {
        auto pano = std::get<0>(stitcher.stitchPano(cv::Size(cols, rows)));
        cv::imwrite((basePath / "pano1.png").string(), pano);
    }

    if (stage < 5)
        return 0;

    // reintegrate by geodesic interpolation of frames between keyframes
    stitcher.reintegrate();
    if (showResults)
    {
        auto pano = std::get<0>(stitcher.stitchPano(cv::Size(cols, rows), false, true));
        cv::imwrite((basePath / "pano2.png").string(), pano);
    }
    stitcher.writeTrafos(basePath / "opt_trafos.yml", WriteType::Readable);

    if (stage < 6)
        return 0;

    // refine all keyframes
    stitcher.refineNonKeyFrames(features, matches::getMatches(kfIntraPath, geomPano), 50);
    stitcher.writeTrafos(basePath / "opt_trafos.bin");
    if (showResults)
    {
        auto pano = std::get<0>(stitcher.stitchPano(cv::Size(cols, rows), false, true));
        cv::imwrite((basePath / "pano3.png").string(), pano);
    }

    stitcher.writeTrafos(basePath / "opt_trafos.yml", WriteType::Readable);
    return 0;
}
