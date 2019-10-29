#include <iostream>
#include <memory>

#include "habitrack/featureContainer.h"
#include "habitrack/imageContainer.h"
#include "habitrack/keyFrameSelector.h"
#include "habitrack/matchesContainer.h"
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
    std::size_t cacheSize = 5;
    auto path = std::string("/home/lars/data/timm/vidAll");

    // load images
    auto imgContainer = std::make_shared<ImageContainer>(path + "/imgs");

    // calc features
    auto ftContainer
        = std::make_shared<FeatureContainer>(imgContainer, path + "/fts", FeatureType::SIFT, 5000);
    ftContainer->compute(cacheSize, ComputeBehavior::Keep);

    // select key frames
    auto keyFrameSelector
        = std::make_unique<KeyFrameSelector>(ftContainer, path + "/key_frames.yml");
    auto keyFrames = keyFrameSelector->compute(0.3, 0.5, ComputeBehavior::Keep);

    // do (exhaustive|mild) matching on key frames only
    auto matchContainer
        = std::make_shared<MatchesContainer>(ftContainer, path + "/kfs", MatchType::Exhaustive, 10,
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
        keyFrames, GeometricType::Homography, Blending::NoBlend);

    stitcher->initTrafos();

    /* auto panoImg0 = std::get<0>(stitcher->stitchPano(cv::Size(1920, 1080), true)); */
    /* drawImg(panoImg0); */

    /* stitcher->globalOptimize(); */
    stitcher->reintegrate();
    auto panoImg1 = std::get<0>(stitcher->stitchPano(cv::Size(1920, 1080), true));
    drawImg(panoImg1);
    /* cv::Mat combinedImg; */
    /* cv::hconcat(panoImg0, panoImg1, combinedImg); */
    /* drawImg(combinedImg); */


    /* stitcher->reintegrate(); */

    /* stitcher->globalOptimize(); */
    /* auto trafoList = sticher->getTrafoList(); */

    // keep keyframe transformation constant
    // init non-keyframes by interpolating
    // bundleadjust
    /* sticher->reintegrate(); */

    return 0;
}
