#include <iostream>
#include <memory>
#include <numeric>

#include "habitrack/featureAggregator.h"
#include "habitrack/featureContainer.h"
#include "habitrack/idTranslator.h"
#include "habitrack/imageAggregator.h"
#include "habitrack/imageContainer.h"
#include "habitrack/keyFrameRecommender.h"
#include "habitrack/keyFrameSelector.h"
#include "habitrack/matchesContainer.h"
#include "habitrack/mildRecommender.h"
#include "habitrack/panoramaStitcher.h"
#include "habitrack/transitions.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace ht;
void drawImg(const cv::Mat& img)
{
    int key = 0;
    do
    {
        cv::imshow("img", img);
        key = cv::waitKey(0);
    } while (key != 27);
}

int main()
{
    std::filesystem::path basePath = "/home/lars/data/ontogenyTest";
    std::vector<std::filesystem::path> videoPaths
        = {basePath / "vid2", basePath / "vid3", basePath / "vid4"};

    // collect img and ft containers and list of keyframes
    std::vector<std::shared_ptr<ImageContainer>> imgContainers;
    std::vector<std::shared_ptr<FeatureContainer>> ftSiftContainers;
    std::vector<std::shared_ptr<FeatureContainer>> ftOrbContainers;
    std::vector<std::vector<std::size_t>> keyFrameList;
    std::vector<std::size_t> sizes;
    std::vector<PairwiseMatches> matchesIntraList;
    std::vector<PairwiseMatches> matchesInterList;

    PairwiseMatches intraMatches;
    PairwiseTrafos intraTrafos;
    for (const auto& path : videoPaths)
    {
        auto imgContainer = std::make_shared<ImageContainer>(path / "imgs");

        // add sift container and keyframes
        auto ftSiftContainer = std::make_shared<FeatureContainer>(
            imgContainer, path / "fts", FeatureType::SIFT, 5000);

        // collect keyframes
        KeyFrameSelector selector(
            ftSiftContainer, GeometricType::Similarity, path / "key_frames.yml");
        auto currKeyFrames = selector.compute(0.3, 0.5, ComputeBehavior::Keep);
        keyFrameList.push_back(currKeyFrames);

        // collect intra and inter matches
        auto matchIntraContainer = std::make_shared<MatchesContainer>(ftSiftContainer,
            path / "kfs/matches_intra", MatchType::Manual, 0,
            GeometricType::Homography | GeometricType::Affinity | GeometricType::Similarity
                | GeometricType::Isometry);
        matchesIntraList.push_back(matchIntraContainer->getMatches(GeometricType::Similarity));

        auto matchInterContainer = std::make_shared<MatchesContainer>(ftSiftContainer,
            path / "kfs/matches_inter", MatchType::Manual, 0,
            GeometricType::Homography | GeometricType::Affinity | GeometricType::Similarity
                | GeometricType::Isometry);
        matchesInterList.push_back(matchInterContainer->getMatches(GeometricType::Similarity));

        ftSiftContainers.push_back(std::move(ftSiftContainer));

        // add and compute orb containers for mild recomender
        auto ftOrbContainer = std::make_shared<FeatureContainer>(
            imgContainer, path / "kfs/fts", FeatureType::ORB, 3000);
        ftOrbContainer->compute(1000, ComputeBehavior::Keep, currKeyFrames);
        ftOrbContainers.push_back(std::move(ftOrbContainer));

        // determine size for mild
        auto currSize = imgContainer->getNumImgs();

        // add size
        sizes.push_back(currSize);
        imgContainers.push_back(std::move(imgContainer));
    }
    Translator translator(sizes);

    // aggregate img and feature container
    auto globalImgContainer = std::make_shared<ImageAggregator>(imgContainers);
    auto globalFtSiftContainer = std::make_shared<FeatureAggregator>(ftSiftContainers);

    // match inter video
    auto recommender = std::make_unique<MildRecommender>(ftOrbContainers);
    auto matchInterVidContainer = std::make_shared<MatchesContainer>(globalFtSiftContainer,
        basePath / "ivlc/matches", MatchType::Strategy, 1, // TODO: incorporate into ML estimator?
        GeometricType::Homography | GeometricType::Affinity | GeometricType::Similarity
            | GeometricType::Isometry,
        std::move(recommender));

    // list of local keyframes to global keyframes
    auto globalKeyFrames = translator.localToGlobal(keyFrameList);
    keyFrameList.clear();
    matchInterVidContainer->compute(5000, ComputeBehavior::Keep, globalKeyFrames);

    // combined local dense matches and inter video sparse matches
    auto interVidMatches = matchInterVidContainer->getMatches(GeometricType::Similarity);
    auto optimalTransitions = transitions::getMostProminantTransition(interVidMatches, sizes);
    std::vector<std::vector<cv::Mat>> localOptimalTrafos;
    for (auto path : videoPaths)
        localOptimalTrafos.push_back(PanoramaStitcher::loadTrafos(path / "kfs/opt_trafos.bin"));

    auto globalInterMatches = translator.localToGlobal(matchesInterList);
    matchesInterList.clear();
    globalInterMatches.insert(std::begin(interVidMatches), std::end(interVidMatches));
    interVidMatches.clear();

    // do pano stitching for every wanted type
    auto stitcher = std::make_unique<PanoramaStitcher>(globalImgContainer, globalFtSiftContainer,
        globalInterMatches, matchInterVidContainer->getTrafos(GeometricType::Similarity),
        globalKeyFrames, GeometricType::Similarity, Blending::NoBlend);

    stitcher->initTrafosFromMultipleVideos(sizes, localOptimalTrafos, optimalTransitions);
    auto panoImg0 = std::get<0>(stitcher->stitchPano(cv::Size(2*1920, 2*1080)));
    drawImg(panoImg0);

    stitcher->globalOptimizeKeyFrames();
    auto panoImg1 = std::get<0>(stitcher->stitchPano(cv::Size(2*1920, 2*1080)));
    drawImg(panoImg1);
    cv::imwrite("combined.png", panoImg1);

    return 0;
}
