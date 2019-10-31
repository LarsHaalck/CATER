#include <iostream>
#include <memory>

#include "habitrack/featureContainer.h"
#include "habitrack/imageContainer.h"
#include "habitrack/keyFrameSelector.h"
#include "habitrack/matchesContainer.h"
#include "habitrack/panoramaStitcher.h"
#include "habitrack/imageAggregator.h"
#include "habitrack/featureAggregator.h"
#include "habitrack/mildRecommender.h"
#include "habitrack/idTranslator.h"

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
int main()
{
    std::filesystem::path basePath = "/home/lars/data/timm";
    std::vector<std::filesystem::path> videoPaths = {
        basePath / "vid1",
        basePath / "vid2",
        basePath / "vid3",
        basePath / "vid4"
    };


    std::vector<std::shared_ptr<ImageContainer>> imgContainers;
    std::vector<std::shared_ptr<FeatureContainer>> ftSiftContainers;
    std::vector<std::shared_ptr<FeatureContainer>> ftOrbContainers;
    std::vector<std::vector<std::size_t>> keyFrameList;
    std::vector<std::size_t> sizes;

    std::size_t minSize = std::numeric_limits<std::size_t>::max();
    for (const auto& path : videoPaths)
    {
        auto imgContainer = std::make_shared<ImageContainer>(path / "imgs");
        auto ftSiftContainer = std::make_shared<FeatureContainer>(
            imgContainer, path / "fts", FeatureType::SIFT, 5000);
        /* ftSiftContainer->compute(50, ComputeBehavior::Keep); // TODO: needed? */

        KeyFrameSelector selector(ftSiftContainer, path / "key_frames.yml");
        auto currKeyFrames = selector.compute(0.3, 0.5, ComputeBehavior::Keep);
        keyFrameList.push_back(currKeyFrames);

        ftSiftContainers.push_back(std::move(ftSiftContainer));

        auto ftOrbContainer = std::make_shared<FeatureContainer>(
            imgContainer, path / "kfs/fts", FeatureType::ORB, 3000);
        ftOrbContainer->compute(1000, ComputeBehavior::Keep, currKeyFrames);
        ftOrbContainers.push_back(std::move(ftOrbContainer));

        auto currSize = imgContainer->getNumImgs();
        minSize = std::min(minSize, currKeyFrames.size());
        sizes.push_back(currSize);
        imgContainers.push_back(std::move(imgContainer));
    }

    std::cout << "min video size is: " << minSize << std::endl;
    auto globalImgContainer = std::make_shared<ImageAggregator>(imgContainers);
    auto globalFtSiftContainer = std::make_shared<FeatureAggregator>(ftSiftContainers);


    auto recommender = std::make_unique<MildRecommender>(ftOrbContainers);
    auto matchContainer = std::make_shared<MatchesContainer>(globalFtSiftContainer,
        basePath / "ivlc/matches", MatchType::Strategy, 0,
        GeometricType::Homography | GeometricType::Affinity | GeometricType::Similarity |
        GeometricType::Isometry, std::move(recommender));

    auto globalKeyFrames = translate::localToGlobal(keyFrameList, sizes);
    matchContainer->compute(5000, ComputeBehavior::Keep, globalKeyFrames);


    // combined local dense matches and inter video sparse matches


    /* // do pano stitching for every wanted type */
    /* auto stitcher = std::make_unique<PanoramaStitcher>(imgContainer, ftContainer, matchContainer, */
    /*     keyFrames, GeometricType::Homography, Blending::NoBlend); */

    /* stitcher->initTrafos(); */

    /* /1* auto panoImg0 = std::get<0>(stitcher->stitchPano(cv::Size(1920, 1080), true)); *1/ */
    /* /1* drawImg(panoImg0); *1/ */

    /* /1* stitcher->globalOptimize(); *1/ */
    /* stitcher->reintegrate(); */
    /* auto panoImg1 = std::get<0>(stitcher->stitchPano(cv::Size(1920, 1080), true)); */
    /* drawImg(panoImg1); */
    /* /1* cv::Mat combinedImg; *1/ */
    /* /1* cv::hconcat(panoImg0, panoImg1, combinedImg); *1/ */
    /* /1* drawImg(combinedImg); *1/ */


    /* /1* stitcher->reintegrate(); *1/ */

    /* /1* stitcher->globalOptimize(); *1/ */
    /* /1* auto trafoList = sticher->getTrafoList(); *1/ */

    /* // keep keyframe transformation constant */
    /* // init non-keyframes by interpolating */
    /* // bundleadjust */
    /* /1* sticher->reintegrate(); *1/ */

    return 0;
}
