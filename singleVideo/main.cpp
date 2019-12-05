#include <iostream>
#include <filesystem>
#include <memory>

#include "habitrack/featureContainer.h"
#include "habitrack/imageContainer.h"
#include "habitrack/keyFrameRecommender.h"
#include "habitrack/keyFrameSelector.h"
#include "habitrack/matchesContainer.h"
#include "habitrack/mildRecommender.h"
#include "habitrack/panoramaStitcher.h"

#include "cxxopts.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

void drawImg(const cv::Mat& img)
{
    int key = 0;
    do
    {
        cv::imshow("img", img);
        key = cv::waitKey(0);
    } while (key != 27);
}

using namespace ht;
namespace fs = std::filesystem;
int main(int argc, char** argv)
{
    fs::path basePath;
    bool showResults = false;
    int stage = 0;
    std::size_t cacheSize = 20;

    int cols = 2 * 1920;
    int rows = 2 * 1080;

    double minCoverage = 0.0;

    cxxopts::Options options("single", "");
    options.add_options()
        ("i,in", "base path containing imgs dir", cxxopts::value(basePath))
        ("v", "show results", cxxopts::value(showResults))
        ("c", "cache", cxxopts::value(cacheSize))
        ("k, cols", "cols", cxxopts::value(cols))
        ("l, rows", "rows", cxxopts::value(rows))
        ("m, min_coverage", "min coverage", cxxopts::value(minCoverage))
        ("s,stage", "0: fts, 1: kfs: 2: matches, 3: init, 4: global, 5: reint; 6: global",
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
    std::cout << "m: " << minCoverage << std::endl;
    std::cout << "s: " << stage << std::endl;

    // load images
    auto imgContainer = std::make_shared<ImageContainer>(basePath / "imgs");

    // calc features
    auto ftContainer
        = std::make_shared<FeatureContainer>(imgContainer, basePath / "fts", FeatureType::SIFT, 5000);
    ftContainer->compute(cacheSize, ComputeBehavior::Keep);

    if (stage < 1)
        return 0;

    // select key frames
    auto keyFrameSelector
        = KeyFrameSelector(ftContainer, GeometricType::Similarity, basePath / "key_frames.yml");
    auto keyFrames = keyFrameSelector.compute(0.3, 0.5, ComputeBehavior::Keep);

    if (stage < 2)
        return 0;

    // calculate matches between images between keyframes via KeyFrameRecommender
    auto keyFrameRecommender = std::make_unique<KeyFrameRecommender>(keyFrames);
    auto matchIntraContainer = std::make_shared<MatchesContainer>(ftContainer,
        basePath / "kfs/matches_intra", MatchType::Strategy, 0,
        GeometricType::Homography | GeometricType::Affinity | GeometricType::Similarity
            | GeometricType::Isometry,
        0, std::move(keyFrameRecommender));
    matchIntraContainer->compute(5000, ComputeBehavior::Keep);

    // calculate matches between keyframes via exhaustive matching
    auto matchInterContainer = std::make_shared<MatchesContainer>(ftContainer,
        basePath / "kfs/matches_inter", MatchType::Exhaustive, 10,
        GeometricType::Homography | GeometricType::Affinity | GeometricType::Similarity
            | GeometricType::Isometry, minCoverage);
    matchInterContainer->compute(5000, ComputeBehavior::Keep, keyFrames);

    // panoramas can only be calculated from theses Types
    GeometricType useableTypes = matchInterContainer->getUsableTypes(keyFrames);
    auto typeList = typeToTypeList(useableTypes);
    std::cout << "Usable types for PanoramaStitcher:" << std::endl;
    for (auto type : typeList)
        std::cout << type << std::endl;

    if (stage < 3)
        return 0;

    // do pano stitching for some wanted type
    auto stitcher = std::make_unique<PanoramaStitcher>(imgContainer, ftContainer,
        matchInterContainer, keyFrames, GeometricType::Similarity, Blending::NoBlend);

    // init trafos of keyframes by concatenating them
    stitcher->initTrafos();
    if (showResults)
    {
        auto pano = std::get<0>(stitcher->stitchPano(cv::Size(cols, rows)));
        cv::imwrite("pano0.png", pano);
        /* drawImg(pano); */
    }

    if (stage < 4)
        return 0;

    // globally optimized these keyframe tranformations and write them for later IVLC
    stitcher->globalOptimizeKeyFrames();
    stitcher->writeTrafos(basePath / "kfs/opt_trafos.bin");
    if (showResults)
    {
        auto pano = std::get<0>(stitcher->stitchPano(cv::Size(cols, rows)));
        cv::imwrite("pano1.png", pano);
        /* drawImg(pano); */
    }

    if (stage < 5)
        return 0;

    // reintegrate by geodesic interpolation of frames between keyframes
    stitcher->reintegrate();
    if (showResults)
    {
        auto pano = std::get<0>(stitcher->stitchPano(cv::Size(cols, rows), true));
        cv::imwrite("pano2.png", pano);
        /* drawImg(pano); */
    }

    if (stage < 6)
        return 0;

    // refine all keyframes
    stitcher->refineNonKeyFrames(matchIntraContainer->getMatches(GeometricType::Similarity), 50);
    stitcher->writeTrafos(basePath / "opt_trafos.bin");
    if (showResults)
    {
        auto pano = std::get<0>(stitcher->stitchPano(cv::Size(cols, rows), true));
        cv::imwrite("pano3.png", pano);
        /* drawImg(pano); */
    }

    stitcher->writeTrafos(basePath / "opt_trafos.yml", WriteType::Readable);
    return 0;
}
