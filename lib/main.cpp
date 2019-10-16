#include <iostream>
#include <memory>

#include "habitrack/imageContainer.h"
#include "habitrack/featureContainer.h"
#include "habitrack/resizeDecorator.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "MILD/loop_closure_detector.h"

using namespace ht;
int main()
{
    std::size_t cacheSize = 200;

    auto container = std::make_shared<ImageContainer>(
        "/home/lars/data/ontogenyTest/vid3/imgs");

    auto ftContainer = std::make_shared<FeatureContainer>(
        container, "/home/lars/data/ontogenyTest/vid3/fts", FeatureType::ORB, 5000);
    ftContainer->compute(cacheSize, false);


    MILD::LoopClosureDetector lcd(FEATURE_TYPE_ORB, 16, 0);
    lcd.displayParameters();

    return 0;
}
