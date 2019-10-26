#include <iostream>
#include <memory>

#include "habitrack/featureContainer.h"
#include "habitrack/imageContainer.h"
#include "habitrack/keyFrameSelector.h"
#include "habitrack/matchesContainer.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace ht;
int main()
{
    std::size_t cacheSize = 5000;
    auto path = std::string("/home/lars/data/ontogenyTest/vid3");

    auto imgContainer = std::make_shared<ImageContainer>(path + "/imgs");

    auto ftContainer
        = std::make_shared<FeatureContainer>(imgContainer, path + "/fts", FeatureType::SIFT, 3000);
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

    auto matchContainer
        = std::make_shared<MatchesContainer>(ftContainer, path + "/matches", MatchType::MILD, 10,
            GeometricType::Homography | GeometricType::Affinity | GeometricType::Similarity);

    matchContainer->compute(cacheSize, ComputeBehavior::Overwrite);
    /* matchContainer->compute(cacheSize, false); */

    return 0;
}
