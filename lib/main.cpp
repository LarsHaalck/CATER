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
    std::size_t cacheSize = 200;
    auto path = std::string("/home/lars/data/ontogenyTest/vid2");

    auto imgContainer = std::make_shared<ImageContainer>(
        path + "/imgs");

    auto ftContainer = std::make_shared<FeatureContainer>(
        imgContainer, path + "/fts", FeatureType::ORB, 5000);
    ftContainer->compute(cacheSize, ComputeBehavior::Keep);


    /* auto matchContainer = std::make_unique<MatchesContainer>(ftContainer, */
    /*         path + "/match", MatchType::Manual, 50, GeometricType::Affinity | GeometricType::Similarity | GeometricType::Isometry); */

    /* auto [trafos, matches] = matchContainer->compute(0, 9); */


    /* auto img0 = imgContainer->at(0); */
    /* auto img1 = imgContainer->at(9); */
    /* auto ft0 = ftContainer->featureAt(0); */
    /* auto ft1 = ftContainer->featureAt(9); */

    /* for (std::size_t i = 1; i < trafos.size(); i++) */
    /* { */
    /*     cv::Mat img; */
    /*     /1* cv::drawMatches(img0, ft0, img1, ft1, matches[i], img); *1/ */
    /*     img1.copyTo(img); */
    /*     cv::warpAffine(img0, img, trafos[i], img0.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT); */
    /*     /1* img1.copyTo(img); *1/ */

    /*     cv::imshow("hi", img); */
    /*     cv::waitKey(0); */
    /* } */


    auto keyFrameSelector = std::make_unique<KeyFrameSelector>(ftContainer, 1920, 1080);
    auto keyFrames = keyFrameSelector->compute(0.3, 0.5);
    /* keyFrameSelector->select( */

    /* std::cout << ftContainer->featureAt(0, ImageType::Regular).size() << std::endl; */
    /* std::cout << ftContainer->descriptorAt(0, ImageType::Regular).size() << std::endl; */

    /* auto matchContainer = std::make_shared<MatchesContainer>( */
    /*     ftContainer, "/home/lars/data/ontogenyTest/vid3/matches", MatchType::Windowed, */
    /*     50, GeometricType::Homography | GeometricType::Similarity); */

    /* matchContainer->compute(cacheSize); */

    /* matchContainer->compute(cacheSize, false); */


    /* MILD::LoopClosureDetector lcd(FEATURE_TYPE_ORB, 16, 0); */
    /* lcd.displayParameters(); */

    return 0;
}

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
