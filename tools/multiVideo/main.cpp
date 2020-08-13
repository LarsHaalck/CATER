#include <iostream>
#include <memory>
#include <numeric>

#include "image-processing/featureAggregator.h"
#include "image-processing/imageAggregator.h"
#include "image-processing/images.h"
#include "image-processing/matches.h"
#include "image-processing/mildRecommender.h"
#include "panorama/idTranslator.h"
#include "panorama/keyFrameRecommender.h"
#include "panorama/keyFrames.h"
#include "panorama/panoramaStitcher.h"
#include "panorama/transitions.h"

#include "cxxopts.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace ht;

int main()
{
    std::filesystem::path basePath = "/home/lars/data/ontogenyTest";
    std::vector<std::filesystem::path> videoPaths
        = {basePath / "vid2", basePath / "vid3", basePath / "vid4"};

    // TODO: deduce feature type from
    bool force = false;
    int cacheSize = 200;

    int cols = 2 * 1920;
    int rows = 2 * 1080;

    GeometricType geomType = GeometricType::Similarity;

    // collect img and ft containers and list of keyframes
    std::vector<Images> imgContainers;
    std::vector<Features> ftLocalContainers;
    std::vector<Features> ftGlobalContainers;
    std::vector<std::vector<std::size_t>> keyFrameList;
    std::vector<std::size_t> sizes;
    std::vector<matches::PairwiseMatches> matchesIntraList;
    std::vector<matches::PairwiseMatches> matchesInterList;

    for (const auto& path : videoPaths)
    {
        auto images = Images(path / "imgs");
        auto ftsLocal = Features::fromDir(images, path / "fts", FeatureType::ORB);
        auto ftsLocalDense
            = Features::fromDir(images, path / "kfs/matches_inter/fts", FeatureType::ORB);

        auto keyFrames = KeyFrames::fromDir(path / "key_frames.yml");
        keyFrameList.push_back(keyFrames);

        auto matchesIntra = matches::getMatches(path / "kfs/matches_intra", geomType);
        matchesIntraList.push_back(std::move(matchesIntra));

        auto matchesInter = matches::getMatches(path / "kfs/matches_inter", geomType);
        matchesInterList.push_back(std::move(matchesInter));

        ftLocalContainers.push_back(std::move(ftsLocal));

        // add and compute orb containers for mild recomender
        auto ftsGlobalPath = path / "kfs/fts";
        auto ftsGlobalType = FeatureType::ORB;
        auto ftsGlobal = Features();
        if (Features::isComputed(images, ftsGlobalPath, ftsGlobalType) && !force)
            ftsGlobal = Features::fromDir(images, ftsGlobalPath, ftsGlobalType, keyFrames);
        else
        {
            ftsGlobal = Features::compute(
                images, ftsGlobalPath, ftsGlobalType, 3000, cacheSize, keyFrames);
        }

        ftGlobalContainers.push_back(std::move(ftsGlobal));
        sizes.push_back(images.size());
        imgContainers.push_back(std::move(images));
    }
    Translator translator(sizes);

    // aggregate img and feature container
    auto combindImgContainer = ImageAggregator(imgContainers);
    auto combinedLocalFtContainer = FeatureAggregator(ftLocalContainers);
    auto combinedGlobalFtContainer = FeatureAggregator(ftGlobalContainers);

    // list of local keyframes to global keyframes
    auto globalKeyFrames = translator.localToGlobal(keyFrameList);

    // match inter video
    auto ivlcMatchPath = basePath / "ivlc/matches";
    auto mildRecommender = std::make_unique<MildRecommender>(ftGlobalContainers);
    if (!matches::isComputed(ivlcMatchPath, geomType) || force)
    {
        matches::compute(ivlcMatchPath, geomType, combinedGlobalFtContainer,
            matches::MatchType::Strategy, 3, 0.0, std::move(mildRecommender), cacheSize,
            globalKeyFrames);
    }

    // combined local dense matches and inter video sparse matches
    auto interVidMatches = matches::getMatches(ivlcMatchPath, geomType);
    auto optimalTransitions = transitions::getMostProminantTransition(interVidMatches, sizes);
    std::vector<std::vector<cv::Mat>> localOptimalTrafos;
    for (auto path : videoPaths)
        localOptimalTrafos.push_back(PanoramaStitcher::loadTrafos(path / "kfs/opt_trafos.bin"));

    auto globalInterMatches = translator.localToGlobal(matchesInterList);
    globalInterMatches.insert(std::begin(interVidMatches), std::end(interVidMatches));

    // do pano stitching for every wanted type
    auto stitcher = PanoramaStitcher(combindImgContainer, globalKeyFrames, geomType);
    /* auto stitcher = std::make_unique<PanoramaStitcher>(globalImgContainer, globalFtSiftContainer, */
    /*     globalInterMatches, matchInterVidContainer->getTrafos(GeometricType::Similarity), */
    /*     globalKeyFrames, GeometricType::Similarity, Blending::NoBlend); */

    stitcher.initTrafosFromMultipleVideos(
        matches::getTrafos(ivlcMatchPath), sizes, localOptimalTrafos, optimalTransitions);
    auto pano = std::get<0>(stitcher.stitchPano(cv::Size(cols, rows)));
    cv::imwrite("combined0.png", pano);

    stitcher.globalOptimizeKeyFrames(combinedGlobalFtContainer, globalInterMatches);
    pano = std::get<0>(stitcher.stitchPano(cv::Size(cols, rows)));
    cv::imwrite("combined1.png", pano);

    stitcher.reintegrate();
    pano = std::get<0>(stitcher.stitchPano(cv::Size(cols, rows)));
    cv::imwrite("combined2.png", pano);

    auto globalIntraMatches = translator.localToGlobal(matchesIntraList);
    stitcher.refineNonKeyFrames(combinedGlobalFtContainer, globalIntraMatches);
    pano = std::get<0>(stitcher.stitchPano(cv::Size(cols, rows)));
    cv::imwrite("combined3.png", pano);

    stitcher.writeTrafos(basePath / "ivlc/opt_trafos.yml", WriteType::Readable);

    return 0;
}
