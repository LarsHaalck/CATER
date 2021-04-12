#include <iostream>
#include <memory>
#include <numeric>


#include "panorama/panoramaEngine.h"
#include "image-processing/images.h"

#include "cxxopts.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <spdlog/spdlog.h>

using namespace ht;

auto ORB = ht::FeatureType::ORB;
namespace fs = std::filesystem;
int main(int argc, const char** argv)
{
    spdlog::set_level(spdlog::level::debug);

    std::vector<fs::path> videoPaths;
    int featureInt = 0;

    bool force = false;
    int cacheSize = 200;

    int cols = 2 * 1920;
    int rows = 2 * 1080;

    int stage = 0;

    cxxopts::Options options("multi", "");
    options.show_positional_help();
    options.add_options()("folders", "folders", cxxopts::value(videoPaths))("g,feature",
        "feature type 0: ORB, 1: SIFT, 2: SuperPoint",
        cxxopts::value(featureInt))("f", "force", cxxopts::value(force))(
        "k, cols", "cols", cxxopts::value(cols))("l, rows", "rows", cxxopts::value(rows))(
        "s,stage", "0: init, 1: opt: 2: refined", cxxopts::value(stage));

    options.parse_positional({"folders"});

    auto result = options.parse(argc, argv);
    if (result.count("folders") < 2)
    {
        std::cout << options.help() << std::endl;
        return -1;
    }

    auto ftType = FeatureType::ORB;
    if (featureInt == 1)
        ftType = FeatureType::SIFT;
    if (featureInt == 2)
        ftType = FeatureType::SuperPoint;

    fs::path basePath = videoPaths[0].parent_path();
    if (basePath.empty())
        basePath = ".";

    PanoramaSettings settings;
    settings.force = force;
    settings.stage = stage;
    settings.cacheSize = cacheSize;
    settings.rows = rows;
    settings.cols = cols;
    settings.ftType = ftType;

    std::vector<Images> imgContainers;
    for (const auto& path : videoPaths)
    {
        auto images = Images(path / "imgs");
        imgContainers.push_back(images);
    }

    PanoramaEngine::stitchMultiPano(imgContainers, videoPaths, basePath, settings);


    /* GeometricType geomType = GeometricType::Similarity; */

    /* // collect img and ft containers and list of keyframes */
    /* std::vector<std::vector<std::size_t>> keyFrameList; */
    /* std::vector<std::size_t> sizes; */
    /* std::vector<PairwiseMatches> matchesIntraList; */
    /* std::vector<PairwiseMatches> matchesInterList; */

    /* std::vector<Features> ftsOrbSparse; */
    /* std::vector<Features> ftsOrbDense; */
    /* std::vector<Features> ftsDense; */

    /* for (const auto& path : videoPaths) */
    /* { */
    /*     auto images = Images(path / "imgs"); */
    /*     auto currFtsOrbSparse = Features::fromDir(images, path / "fts", ORB); */
    /*     auto currFtsOrbDense = Features::fromDir(images, path / "kfs/fts", ORB); */

    /*     if (ftType != ORB) */
    /*         ftsDense.push_back(Features::fromDir(images, path / "kfs/fts", ftType)); */

    /*     auto keyFrames = KeyFrames::fromDir(path / "key_frames.yml"); */
    /*     keyFrameList.push_back(keyFrames); */

    /*     auto matchesIntra = matches::getMatches(path / "kfs/maches_intra", geomType); */
    /*     matchesIntraList.push_back(std::move(matchesIntra)); */

    /*     auto matchesInter = matches::getMatches(path / "kfs/maches_inter", geomType); */
    /*     matchesInterList.push_back(std::move(matchesInter)); */

    /*     ftsOrbSparse.push_back(std::move(currFtsOrbSparse)); */
    /*     ftsOrbDense.push_back(std::move(currFtsOrbDense)); */

    /*     sizes.push_back(images.size()); */
    /*     imgContainers.push_back(std::move(images)); */
    /* } */
    /* Translator translator(sizes); */

    /* // aggregate img and feature container */
    /* auto combinedImgContainer = ImageAggregator(imgContainers); */
    /* auto combinedSparseFtContainer = FeatureAggregator(ftsOrbSparse); */
    /* auto combinedDenseOrbFtContainer = FeatureAggregator(ftsOrbDense); */
    /* auto combinedDenseFtContainer = FeatureAggregator(ftsDense); */

    /* // list of local keyframes to global keyframes */
    /* auto globalKeyFrames = translator.localToGlobal(keyFrameList); */

    /* // match inter video */
    /* auto ivlcMatchPath = basePath / "ivlc/matches"; */
    /* auto mildRecommender = std::make_unique<MildRecommender>(combinedDenseOrbFtContainer); */
    /* if (!matches::isComputed(ivlcMatchPath, geomType) || force) */
    /* { */
    /*     if (ftType != ORB) */
    /*     { */
    /*         matches::compute(ivlcMatchPath, geomType, combinedDenseFtContainer, */
    /*             matches::MatchType::Strategy, 10, 0.0, std::move(mildRecommender), cacheSize, */
    /*             globalKeyFrames); */
    /*     } */
    /*     else */
    /*     { */
    /*         matches::compute(ivlcMatchPath, geomType, combinedDenseOrbFtContainer, */
    /*             matches::MatchType::Strategy, 10, 0.0, std::move(mildRecommender), cacheSize, */
    /*             globalKeyFrames); */
    /*     } */
    /* } */

    /* // combined local dense matches and inter video sparse matches */
    /* auto interVidMatches = matches::getMatches(ivlcMatchPath, geomType); */
    /* auto optimalTransitions = getMostProminantTransition(interVidMatches, sizes); */
    /* std::vector<std::vector<cv::Mat>> localOptimalTrafos; */
    /* for (auto path : videoPaths) */
    /*     localOptimalTrafos.push_back(PanoramaStitcher::getTrafos(path / "kfs/opt_trafos.bin")); */

    /* auto globalInterMatches = translator.localToGlobal(matchesInterList); */
    /* globalInterMatches.insert(std::begin(interVidMatches), std::end(interVidMatches)); */

    /* auto stitcher = PanoramaStitcher(combinedImgContainer, globalKeyFrames, geomType); */
    /* stitcher.initTrafosFromMultipleVideos( */
    /*     matches::getTrafos(ivlcMatchPath, geomType), sizes, localOptimalTrafos, optimalTransitions); */

    /* auto pano = std::get<0>(stitcher.stitchPano(cv::Size(cols, rows))); */
    /* cv::imwrite(basePath / "combined0.png", pano); */

    /* if (ftType != ORB) */
    /*     stitcher.globalOptimizeKeyFrames(combinedDenseFtContainer, globalInterMatches); */
    /* else */
    /*     stitcher.globalOptimizeKeyFrames(combinedDenseOrbFtContainer, globalInterMatches); */

    /* pano = std::get<0>(stitcher.stitchPano(cv::Size(cols, rows))); */
    /* cv::imwrite(basePath / "combined1.png", pano); */

    /* stitcher.reintegrate(); */
    /* pano = std::get<0>( */
    /*     stitcher.stitchPano(cv::Size(cols, rows), false, basePath / "reint_centers.yml")); */
    /* cv::imwrite(basePath / "combined2.png", pano); */

    /* auto globalIntraMatches = translator.localToGlobal(matchesIntraList); */
    /* stitcher.refineNonKeyFrames(combinedSparseFtContainer, globalIntraMatches); */
    /* pano = std::get<0>( */
    /*     stitcher.stitchPano(cv::Size(cols, rows), false, basePath / "opt_centers.yml")); */
    /* cv::imwrite(basePath / "combined3.png", pano); */

    /* stitcher.writeTrafos(basePath / "ivlc/opt_trafos.yml", WriteType::Readable); */

    return 0;
}
