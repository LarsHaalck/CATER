#include <iostream>
#include <memory>
#include <numeric>

#include "image-processing/featureAggregator.h"
#include "image-processing/imageAggregator.h"
#include "image-processing/images.h"
#include "image-processing/matches.h"
#include "image-processing/mildRecommender.h"
#include "panorama/keyFrameRecommender.h"
#include "panorama/keyFrames.h"
#include "panorama/panoramaStitcher.h"
#include "panorama/idTranslator.h"

/* #include "cxxopts.hpp" */
/* #include "habitrack/featureAggregator.h" */
/* #include "habitrack/featureContainer.h" */
/* #include "habitrack/idTranslator.h" */
/* #include "habitrack/imageAggregator.h" */
/* #include "habitrack/imageContainer.h" */
/* #include "habitrack/keyFrameRecommender.h" */
/* #include "habitrack/keyFrameSelector.h" */
/* #include "habitrack/matchesContainer.h" */
/* #include "habitrack/mildRecommender.h" */
/* #include "habitrack/panoramaStitcher.h" */
/* #include "habitrack/transitions.h" */

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace ht;

int main()
{
    std::filesystem::path basePath = "/home/lars/data/ontogenyTest";
    std::vector<std::filesystem::path> videoPaths
        = {basePath / "vid2", basePath / "vid3", basePath / "vid4"};

    bool force = false;
    int cacheSize = 200;

    // collect img and ft containers and list of keyframes
    std::vector<Images> imgContainers;
    std::vector<Features> ftLocalContainers;
    std::vector<Features> ftGlobalContainers;
    std::vector<std::vector<std::size_t>> keyFrameList;
    std::vector<std::size_t> sizes;
    std::vector<matches::PairwiseMatches> matchesIntraList;
    std::vector<matches::PairwiseMatches> matchesInterList;

    GeometricType geomType = GeometricType::Similarity;
    for (const auto& path : videoPaths)
    {
        auto images = Images(path / "imgs");
        auto ftsLocal = Features::fromDir(images, path / "fts", FeatureType::SIFT);

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
    auto globalImgContainer = ImageAggregator(imgContainers);
    auto globalFtSiftContainer = FeatureAggregator(ftLocalContainers);


    // match inter video
    /* auto ivlcMatchPath = basePath / "ivlc/matches"; */
    /* auto mildRecommender = MildRecommender(ftGlobalContainers); */
    /* if (!matches::isComputed(ivlcMatchPath, geomType) || force) */
    /* { */
    /*     matches::compute(ivlcMatchPath, geomType, global, matches::MatchType::Strategy, 3, 0.0, */
    /*         std::move(mildRecommender), cacheSize, keyFrames); */
    /* } */

    /* auto matchInterVidContainer = std::make_shared<MatchesContainer>(globalFtSiftContainer, */
    /*     basePath / "ivlc/matches", MatchType::Strategy, 1, // TODO: incorporate into ML estimator? */
    /*     GeometricType::Homography | GeometricType::Affinity | GeometricType::Similarity */
    /*         | GeometricType::Isometry, */
    /*     0, std::move(recommender)); */

    /* // list of local keyframes to global keyframes */
    /* auto globalKeyFrames = translator.localToGlobal(keyFrameList); */
    /* keyFrameList.clear(); */
    /* matchInterVidContainer->compute(5000, ComputeBehavior::Keep, globalKeyFrames); */

    /* // combined local dense matches and inter video sparse matches */
    /* auto interVidMatches = matchInterVidContainer->getMatches(GeometricType::Similarity); */
    /* auto optimalTransitions = transitions::getMostProminantTransition(interVidMatches, sizes); */
    /* std::vector<std::vector<cv::Mat>> localOptimalTrafos; */
    /* for (auto path : videoPaths) */
    /*     localOptimalTrafos.push_back(PanoramaStitcher::loadTrafos(path / "kfs/opt_trafos.bin")); */

    /* auto globalInterMatches = translator.localToGlobal(matchesInterList); */
    /* matchesInterList.clear(); */
    /* globalInterMatches.insert(std::begin(interVidMatches), std::end(interVidMatches)); */
    /* interVidMatches.clear(); */

    /* // do pano stitching for every wanted type */
    /* auto stitcher = std::make_unique<PanoramaStitcher>(globalImgContainer, globalFtSiftContainer, */
    /*     globalInterMatches, matchInterVidContainer->getTrafos(GeometricType::Similarity), */
    /*     globalKeyFrames, GeometricType::Similarity, Blending::NoBlend); */

    /* stitcher->initTrafosFromMultipleVideos(sizes, localOptimalTrafos, optimalTransitions); */
    /* auto panoImg0 = std::get<0>(stitcher->stitchPano(cv::Size(2 * 1920, 2 * 1080))); */
    /* /1* drawImg(panoImg0); *1/ */
    /* cv::imwrite("combined0.png", panoImg0); */

    /* stitcher->globalOptimizeKeyFrames(); */
    /* auto panoImg1 = std::get<0>(stitcher->stitchPano(cv::Size(2 * 1920, 2 * 1080))); */
    /* /1* drawImg(panoImg1); *1/ */
    /* cv::imwrite("combined1.png", panoImg1); */

    /* stitcher->reintegrate(); */
    /* auto panoImg2 = std::get<0>(stitcher->stitchPano(cv::Size(2 * 1920, 2 * 1080), true)); */
    /* cv::imwrite("combined2.png", panoImg2); */

    /* auto globalIntraMatches = translator.localToGlobal(matchesIntraList); */
    /* matchesIntraList.clear(); */
    /* stitcher->refineNonKeyFrames(globalIntraMatches); */
    /* auto panoImg3 = std::get<0>(stitcher->stitchPano(cv::Size(2 * 1920, 2 * 1080), true)); */
    /* cv::imwrite("combined3.png", panoImg3); */

    /* stitcher->writeTrafos(basePath / "ivlc/opt_trafos.yml", WriteType::Readable); */

    /* return 0; */
}
