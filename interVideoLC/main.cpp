#include <iostream>
#include <memory>
#include <numeric>

#include "habitrack/featureAggregator.h"
#include "habitrack/featureContainer.h"
#include "habitrack/idTranslator.h"
#include "habitrack/imageAggregator.h"
#include "habitrack/imageContainer.h"
#include "habitrack/keyFrameSelector.h"
#include "habitrack/matchesContainer.h"
#include "habitrack/mildRecommender.h"
#include "habitrack/panoramaStitcher.h"
#include "habitrack/keyFrameRecommender.h"

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

/* void getMostProminantTransition( */
/*     const PairwiseMatches& matches, const std::vector<std::size_t>& sizes) */
/* { */
/*     std::vector<std::size_t> cumSizes(sizes.size(), 0); */
/*     std::partial_sum(std::begin(sizes), std::end(sizes) - 1, std::begin(cumSizes) + 1); */

/*     auto keys = MatchesContainer::getKeyList(matches); */

/*     auto getBlock = [cumSizes](std::size_t idx) { */
/*         for (int i = static_cast<int>(cumSizes.size()) - 1; i >= 0; i--) */
/*         { */
/*             if (idx >= cumSizes[i]) */
/*                 return static_cast<std::size_t>(i); */
/*         } */
/*         return static_cast<std::size_t>(0); */
/*     }; */

/*     std::unordered_map<std::pair<std::size_t, std::size_t>, */
/*         std::tuple<std::size_t, std::size_t, std::size_t>> */
/*         transitions; */
/*     for (auto& pair : keys) */
/*     { */
/*         auto [idI, idJ] = pair; */
/*         auto blockI = getBlock(idI); */
/*         auto blockJ = getBlock(idJ); */
/*         auto blockPair = std::make_pair(blockI, blockJ); */

/*         if (!transitions.count(blockPair)) */
/*             transitions[blockPair] = std::make_tuple(idI, idJ, matches.at(pair).size()); */
/*         else */
/*         { */
/*             auto [oldI, oldJ, count] = transitions[blockPair]; */
/*             if (matches.at(pair).size() > count) */
/*                 transitions[blockPair] = std::make_tuple(idI, idJ, matches.at(pair).size()); */
/*         } */
/*     } */

/*     for (auto elem : transitions) */
/*     { */
/*         auto [blockI, blockJ] = elem.first; */
/*         auto [idI, idJ, count] = elem.second; */
/*         std::cout << "Transition from block: " << blockI << " --> " << blockJ << std::endl; */
/*         std::cout << "With frames: " << idI << " --> " << idJ << ", count = " << count <<
 * std::endl; */
/*         std::cout << "--------------------------------------------------" << std::endl; */
/*     } */
/* } */

int main()
{
    std::filesystem::path basePath = "/home/lars/data/ontogenyTest";
    std::vector<std::filesystem::path> videoPaths
        = {basePath / "vid3", basePath / "vid4"};
        /* = {basePath / "vid1", basePath / "vid2", basePath / "vid3", basePath / "vid4"}; */

    std::vector<std::shared_ptr<ImageContainer>> imgContainers;
    std::vector<std::shared_ptr<FeatureContainer>> ftSiftContainers;

    /* std::vector<std::shared_ptr<FeatureContainer>> ftOrbContainers; */
    /* std::vector<std::vector<std::size_t>> keyFrameList; */
    std::vector<std::size_t> sizes;

    /* std::size_t minSize = std::numeric_limits<std::size_t>::max(); */
    for (const auto& path : videoPaths)
    {
        auto imgContainer = std::make_shared<ImageContainer>(path / "imgs");

        // add sift container and keyframes
        auto ftSiftContainer = std::make_shared<FeatureContainer>(
            imgContainer, path / "fts", FeatureType::SIFT, 5000);

        /* KeyFrameSelector selector(ftSiftContainer, GeometricType::Similarity, path / */
        /*     "key_frames.yml"); */
        /* auto currKeyFrames = selector.compute(0.3, 0.5, ComputeBehavior::Keep); */
        /* keyFrameList.push_back(currKeyFrames); */
        ftSiftContainers.push_back(std::move(ftSiftContainer));

        // add and compute orb containers for mild recomender
        /* auto ftOrbContainer = std::make_shared<FeatureContainer>( */
        /*     imgContainer, path / "kfs/fts", FeatureType::ORB, 3000); */
        /* ftOrbContainer->compute(1000, ComputeBehavior::Keep, currKeyFrames); */
        /* ftOrbContainers.push_back(std::move(ftOrbContainer)); */

        // determine size for mild
        auto currSize = imgContainer->getNumImgs();
        /* minSize = std::min(minSize, currKeyFrames.size()); */

        // add size
        sizes.push_back(currSize);
        imgContainers.push_back(std::move(imgContainer));
    }

    /* // aggregate img and feature container */
    auto globalImgContainer = std::make_shared<ImageAggregator>(imgContainers);
    auto globalFtSiftContainer = std::make_shared<FeatureAggregator>(ftSiftContainers);

    auto keyFrameSelector = std::make_unique<KeyFrameSelector>(
        globalFtSiftContainer, GeometricType::Similarity, basePath / "all/key_frames.yml");
    auto keyFrames = keyFrameSelector->compute(0.3, 0.5, ComputeBehavior::Keep);

    auto keyFrameRecommender = std::make_unique<KeyFrameRecommender>(keyFrames);
    auto kfsMatchContainer = std::make_shared<MatchesContainer>(globalFtSiftContainer,
        basePath / "all/matches_kfs", MatchType::Strategy, 0,
        GeometricType::Homography | GeometricType::Affinity | GeometricType::Similarity
            | GeometricType::Isometry, std::move(keyFrameRecommender));
    kfsMatchContainer->compute(5000, ComputeBehavior::Keep);

    auto exMatchContainer = std::make_shared<MatchesContainer>(globalFtSiftContainer,
        basePath / "all/matches", MatchType::Exhaustive, 0,
        GeometricType::Homography | GeometricType::Affinity | GeometricType::Similarity
            | GeometricType::Isometry);
    exMatchContainer->compute(5000, ComputeBehavior::Keep, keyFrames);
    auto usableTypes = exMatchContainer->getUsableTypes(keyFrames);
    auto typeList = typeToTypeList(usableTypes);
    for (auto type : typeList)
        std::cout << type << std::endl;


    auto type = GeometricType::Similarity;
    auto matches = exMatchContainer->getMatches(type);
    auto trafos = exMatchContainer->getTrafos(type);


    auto matchesAdd = kfsMatchContainer->getMatches(type);
    /* auto trafosAff = kfsMatchContainer->getTrafos(type); */
    matches.insert(std::begin(matchesAdd), std::end(matchesAdd));



    auto stitcher = std::make_unique<PanoramaStitcher>(globalImgContainer, globalFtSiftContainer,
        matches, trafos, keyFrames, type, Blending::NoBlend);
    stitcher->initTrafos();
    /* auto panoImg0 = std::get<0>(stitcher->stitchPano(cv::Size(1920, 1080))); */

    stitcher->globalOptimize();
    stitcher->reintegrate();
    auto panoImg0 = std::get<0>(stitcher->stitchPano(cv::Size(1920, 1080), true));
    stitcher->globalOptimize(FramesMode::AllFrames, KeyFramesMode::Fixed);
    auto panoImg1 = std::get<0>(stitcher->stitchPano(cv::Size(1920, 1080), true));
    stitcher->globalOptimize(FramesMode::AllFrames, KeyFramesMode::Variable);
    auto panoImg2 = std::get<0>(stitcher->stitchPano(cv::Size(1920, 1080), true));

    cv::Mat combinedImg;
    cv::hconcat(panoImg0, panoImg1, combinedImg);
    cv::hconcat(combinedImg, panoImg2, combinedImg);
    drawImg(combinedImg);
    /* auto panoImg0 = std::get<0>(stitcher->stitchPano(cv::Size(1920, 1080), true)); */
    /* drawImg(panoImg0); */

    /* auto panoImg1 = std::get<0>(stitcher->stitchPano(cv::Size(1920, 1080), true)); */
    /* drawImg(panoImg1); */




    /* // match inter video */
    /* auto recommender = std::make_unique<MildRecommender>(ftOrbContainers); */
    /* auto matchContainer = std::make_shared<MatchesContainer>(globalFtSiftContainer, */
    /*     basePath / "ivlc/matches", MatchType::Strategy, 1, // TODO: incorporate into ML estimator
     */
    /*     GeometricType::Homography | GeometricType::Affinity | GeometricType::Similarity | */
    /*     GeometricType::Isometry, std::move(recommender)); */
    /* auto globalKeyFrames = translate::localToGlobal(keyFrameList, sizes); */
    /* matchContainer->compute(5000, ComputeBehavior::Keep, globalKeyFrames); */

    /* // combined local dense matches and inter video sparse matches */
    /* auto interMatches = matchContainer->getMatches(GeometricType::Similarity); */
    /* getMostProminantTransition(interMatches, sizes); */

    /* /1* // do pano stitching for every wanted type *1/ */
    /* /1* auto stitcher = std::make_unique<PanoramaStitcher>(globalImgContainer,
     * globalFtSiftContainer, *1/ */
    /* /1*     matchContainer, globalKeyFrames, GeometricType::Homography, Blending::NoBlend); *1/
     */

    /* /1* stitcher->initTrafos(); *1/ */

    /* /1* /2* auto panoImg0 = std::get<0>(stitcher->stitchPano(cv::Size(1920, 1080), true)); *2/
     * *1/ */
    /* /1* /2* drawImg(panoImg0); *2/ *1/ */

    /* /1* /2* stitcher->globalOptimize(); *2/ *1/ */
    /* /1* stitcher->reintegrate(); *1/ */
    /* /1* auto panoImg1 = std::get<0>(stitcher->stitchPano(cv::Size(1920, 1080), true)); *1/ */
    /* /1* drawImg(panoImg1); *1/ */
    /* /1* /2* cv::Mat combinedImg; *2/ *1/ */
    /* /1* /2* cv::hconcat(panoImg0, panoImg1, combinedImg); *2/ *1/ */
    /* /1* /2* drawImg(combinedImg); *2/ *1/ */

    /* /1* /2* stitcher->reintegrate(); *2/ *1/ */

    /* /1* /2* stitcher->globalOptimize(); *2/ *1/ */
    /* /1* /2* auto trafoList = sticher->getTrafoList(); *2/ *1/ */

    /* /1* // keep keyframe transformation constant *1/ */
    /* /1* // init non-keyframes by interpolating *1/ */
    /* /1* // bundleadjust *1/ */
    /* /1* /2* sticher->reintegrate(); *2/ *1/ */

    /* return 0; */
}
