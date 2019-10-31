#include <iostream>
#include <memory>

#include "habitrack/featureContainer.h"
#include "habitrack/imageContainer.h"
#include "habitrack/keyFrameSelector.h"
#include "habitrack/matchesContainer.h"
#include "habitrack/panoramaStitcher.h"

#include "habitrack/imageAggregator.h"
#include "habitrack/featureAggregator.h"

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
    std::vector<std::shared_ptr<FeatureContainer>> ftContainers;
    std::vector<std::vector<std::size_t>> keyFrameList;
    for (const auto& path : videoPaths)
    {
        auto imgContainer = std::make_shared<ImageContainer>(path / "imgs");
        auto ftContainer = std::make_shared<FeatureContainer>(
            imgContainer, path / "fts", FeatureType::SIFT, 5000);

        KeyFrameSelector selector(ftContainer, path / "key_frames.yml");

        keyFrameList.push_back(selector.compute(0.3, 0.5, ComputeBehavior::Keep));
        imgContainers.push_back(std::move(imgContainer));
        ftContainers.push_back(std::move(ftContainer));
    }

    auto globalImgContainer = std::make_shared<ImageAggregator>(imgContainers);
    auto globalFtContainer = std::make_shared<FeatureAggregator>(ftContainers);

    /* std::cout << globalImgContainer->getNumImgs() << std::endl; */

    /* auto cache = globalImgContainer->getCache(2, {0, 50, 100, 120, 140, 180, 200}); */

    /* std::cout << cache->getNumChunks() << std::endl; */
    /* for (std::size_t i = 0; i < cache->getNumChunks(); i++) */
    /* { */
    /*     std::cout << cache->getChunkSize(i) << std::endl; */
    /*     auto chunk = cache->getChunk(i); */
    /*     for (std::size_t k = 0; k < cache->getChunkSize(i); k++) */
    /*     { */
    /*         drawImg(chunk[k]); */
    /*     } */
    /* } */

    /* std::size_t cacheSize = 5; */
    /* auto path = std::string("/home/lars/data/timm/vid4"); */

    /* // load images */
    /* auto imgContainer = std::make_shared<ImageContainer>(path + "/imgs"); */

    /* // calc features */
    /* auto ftContainer */
    /*     = std::make_shared<FeatureContainer>(imgContainer, path + "/fts", FeatureType::SIFT, 5000); */
    /* ftContainer->compute(cacheSize, ComputeBehavior::Keep); */

    /* // select key frames */
    /* auto keyFrameSelector */
    /*     = std::make_unique<KeyFrameSelector>(ftContainer, path + "/key_frames.yml"); */
    /* auto keyFrames = keyFrameSelector->compute(0.3, 0.5, ComputeBehavior::Keep); */

    /* // do (exhaustive|mild) matching on key frames only */
    /* auto matchContainer */
    /*     = std::make_shared<MatchesContainer>(ftContainer, path + "/kfs", MatchType::Exhaustive, 10, */
    /*         GeometricType::Homography | GeometricType::Affinity | GeometricType::Similarity */
    /*             | GeometricType::Isometry); */
    /* matchContainer->compute(5000, ComputeBehavior::Keep, keyFrames); */

    /* GeometricType useableTypes = matchContainer->getUsableTypes(keyFrames); */
    /* return 0; */

    /* auto typeList = typeToTypeList(useableTypes); */
    /* for (auto type : typeList) */
    /* { */
    /*     std::cout << type << std::endl; */
    /* } */


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
