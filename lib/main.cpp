#include <iostream>
#include <memory>

#include "habitrack/imageContainer.h"
#include "habitrack/featureContainer.h"
#include "habitrack/matchesContainer.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "MILD/loop_closure_detector.h"

using namespace ht;
int main()
{
    std::size_t cacheSize = 200;

    auto imgContainer = std::make_shared<ImageContainer>(
        "/home/lars/data/ontogenyTest/vid2/imgs");

    auto ftContainer = std::make_shared<FeatureContainer>(
        imgContainer, "/home/lars/data/ontogenyTest/vid2/fts", FeatureType::ORB, 5000);
    ftContainer->compute(cacheSize, ComputeBehavior::Keep);

    /* auto keyFrameSelector = std::make_unique<KeyFrameSelector>( */
    /*     imgContainer, ftContainer); */
    /* keyFrameSelector->select( */

    /* std::cout << ftContainer->featureAt(0, ImageType::Regular).size() << std::endl; */
    /* std::cout << ftContainer->descriptorAt(0, ImageType::Regular).size() << std::endl; */

    auto matchContainer = std::make_shared<MatchesContainer>(
        ftContainer, "/home/lars/data/ontogenyTest/vid3/matches", MatchType::MILD,
        50, GeometricType::Homography | GeometricType::Similarity);

    matchContainer->compute(cacheSize);

    /* matchContainer->compute(cacheSize, false); */


    /* MILD::LoopClosureDetector lcd(FEATURE_TYPE_ORB, 16, 0); */
    /* lcd.displayParameters(); */

    return 0;
}
