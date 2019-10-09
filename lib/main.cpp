#include <iostream>
#include <memory>

#include "habitrack/imageContainer.h"
#include "habitrack/featureContainer.h"
#include "habitrack/resizeDecorator.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace ht;
int main()
{
    std::size_t cacheSize = 10;

    auto container = std::make_shared<ImageContainer>(
        "/home/lars/data/ontogenyTest/vid1/imgs");

    auto ftContainer = std::make_shared<FeatureContainer>(
        container, "/home/lars/data/ontogenyTest/vid1/fts", FeatureType::ORB, 5000);
    ftContainer->compute(true, cacheSize);

    return 0;
}
