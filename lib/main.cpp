#include <iostream>
#include <memory>

#include "habitrack/imageContainer.h"
#include "habitrack/featureContainer.h"
#include "habitrack/matchesContainer.h"
#include "habitrack/keyFrameSelector.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


using namespace ht;
int main()
{
    std::size_t cacheSize = 50;
    auto path = std::string("/home/lars/data/ontogenyTest/vid3");

    auto imgContainer = std::make_shared<ImageContainer>(
        path + "/imgs");

    auto ftContainer = std::make_shared<FeatureContainer>(
        imgContainer, path + "/fts", FeatureType::SIFT, 3000);
    ftContainer->compute(cacheSize, ComputeBehavior::Keep);

    /* auto keyFrameSelector = std::make_unique<KeyFrameSelector>(ftContainer); */
    /* auto keyFrames = keyFrameSelector->compute(0.3, 0.75); */

    /* for (auto kf : keyFrames) */
    /* { */
    /*     std::cout << kf << std::endl; */
    /*     cv::imshow("hi", imgContainer->at(kf)); */
    /*     cv::waitKey(0); */
    /* } */

    /* std::cout << std::endl; */

    auto matchContainer = std::make_shared<MatchesContainer>(
        ftContainer, "/home/lars/data/ontogenyTest/vid3/matches", MatchType::Windowed,
        50, GeometricType::Homography);

    matchContainer->compute(cacheSize);
    /* matchContainer->compute(cacheSize, false); */


    return 0;
}

    /* auto matchContainer = std::make_unique<MatchesContainer>(ftContainer, */
    /*         path + "/match", MatchType::Manual, 50, */
    /*         GeometricType::Homography | GeometricType::Affinity | GeometricType::Similarity | GeometricType::Isometry); */

    /* size_t idI = 0; */
    /* size_t idJ = 9; */
    /* auto [trafos, matches] = matchContainer->compute(idI, idJ); */
    /* auto img0 = imgContainer->at(idI); */
    /* auto img1 = imgContainer->at(idJ); */
    /* auto ft0 = ftContainer->featureAt(idI); */
    /* auto ft1 = ftContainer->featureAt(idJ); */

    /* for (std::size_t i = 0; i < trafos.size(); i++) */
    /* { */
    /*     std::cout << trafos[i] << std::endl; */
    /*     cv::Mat img; */
    /*     cv::drawMatches(img0, ft0, img1, ft1, matches[i], img); */
    /*     /1* img1.copyTo(img); *1/ */
    /*     /1* cv::warpAffine(img0, img, trafos[i], img0.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT); *1/ */
    /*     /1* img1.copyTo(img); *1/ */

    /*     cv::imshow("hi", img); */
    /*     cv::waitKey(0); */
    /* } */




    /* auto cache = imgContainer->getCache(2, {0, 10, 20, 40, 100}); */
    /* for (std::size_t i = 0; i < cache->getNumChunks(); i++) */
    /* { */
    /*     std::cout << "getting new chunk" << std::endl; */
    /*     auto chunk = cache->getChunk(i); */
    /*     for (std::size_t j = 0; j < chunk.size(); j++) */
    /*     { */
    /*         cv::imshow("hi", chunk[j]); */
    /*         cv::waitKey(0); */
    /*     } */
    /* } */
