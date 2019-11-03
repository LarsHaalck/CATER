#include <iostream>
#include <memory>

#include "habitrack/featureContainer.h"
#include "habitrack/imageContainer.h"
#include "habitrack/keyFrameRecommender.h"
#include "habitrack/keyFrameSelector.h"
#include "habitrack/matchesContainer.h"
#include "habitrack/mildRecommender.h"
#include "habitrack/panoramaStitcher.h"

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
    std::size_t cacheSize = 20;
    auto path = std::string("/home/lars/data/ontogenyTest/vid4");

    // load images
    auto imgContainer = std::make_shared<ImageContainer>(path + "/imgs");

    // calc features
    auto ftContainer
        = std::make_shared<FeatureContainer>(imgContainer, path + "/fts", FeatureType::SIFT, 3000);
    ftContainer->compute(cacheSize, ComputeBehavior::Keep);

    // select key frames
    auto keyFrameSelector = std::make_unique<KeyFrameSelector>(
        ftContainer, GeometricType::Similarity, path + "/key_frames.yml");
    auto keyFrames = keyFrameSelector->compute(0.3, 0.5, ComputeBehavior::Keep);

    auto keyFrameRecommender = std::make_unique<KeyFrameRecommender>(keyFrames);
    auto matchContainer = std::make_shared<MatchesContainer>(ftContainer,
        path + "/kfs/matches_intra", MatchType::Strategy, 0,
        GeometricType::Homography | GeometricType::Affinity | GeometricType::Similarity
            | GeometricType::Isometry,
        std::move(keyFrameRecommender));
    matchContainer->compute(5000, ComputeBehavior::Keep);

    /* for (auto& elem : keyFrames) */
    /*     std::cout << elem << std::endl; */
    // do (exhaustive|mild) matching on key frames only
    matchContainer = std::make_shared<MatchesContainer>(ftContainer, path + "/kfs/matches_inter",
        MatchType::Exhaustive, 10,
        GeometricType::Homography | GeometricType::Affinity | GeometricType::Similarity
            | GeometricType::Isometry);
    matchContainer->compute(5000, ComputeBehavior::Keep, keyFrames);

    GeometricType useableTypes = matchContainer->getUsableTypes(keyFrames);

    auto typeList = typeToTypeList(useableTypes);
    for (auto type : typeList)
    {
        std::cout << type << std::endl;
    }

    // do pano stitching for every wanted type
    auto stitcher = std::make_unique<PanoramaStitcher>(imgContainer, ftContainer, matchContainer,
        keyFrames, GeometricType::Similarity, Blending::NoBlend);

    stitcher->initTrafos();

    auto panoImg0 = std::get<0>(stitcher->stitchPano(cv::Size(1920, 1080)));
    /* drawImg(panoImg0); */

    stitcher->globalOptimize();
    /* stitcher->reintegrate(); */
    auto panoImg1 = std::get<0>(stitcher->stitchPano(cv::Size(1920, 1080)));
    drawImg(panoImg1);
    cv::Mat combinedImg;
    cv::hconcat(panoImg0, panoImg1, combinedImg);
    drawImg(combinedImg);

    /* stitcher->reintegrate(); */

    /* stitcher->globalOptimize(); */
    /* auto trafoList = sticher->getTrafoList(); */

    // keep keyframe transformation constant
    // init non-keyframes by interpolating
    // bundleadjust
    /* sticher->reintegrate(); */

    return 0;
}
