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
    std::size_t cacheSize = 50;
    auto path = std::string("/home/lars/data/ontogenyTest/vid3");

    // load images
    auto imgContainer = std::make_shared<ImageContainer>(path + "/imgs");

    // calc features
    auto ftContainer
        = std::make_shared<FeatureContainer>(imgContainer, path + "/fts", FeatureType::SIFT, 5000);
    ftContainer->compute(cacheSize, ComputeBehavior::Keep);

    // select key frames
    auto keyFrameSelector
        = std::make_unique<KeyFrameSelector>(ftContainer, path + "/key_frames.yml");
    auto keyFrames = keyFrameSelector->compute(0.3, 0.75, ComputeBehavior::Keep);

    // do (exhaustive|mild) matching on key frames only
    auto matchContainer
        = std::make_shared<MatchesContainer>(ftContainer, path + "/kfs", MatchType::MILD, 10,
            GeometricType::Homography | GeometricType::Affinity | GeometricType::Similarity
                | GeometricType::Isometry);
    matchContainer->compute(5000, ComputeBehavior::Keep, keyFrames);

    GeometricType useableTypes = matchContainer->getUsableTypes(keyFrames);

    auto typeList = typeToTypeList(useableTypes);
    for (auto type : typeList)
    {
        std::cout << type << std::endl;
    }

    /* auto matches = matchContainer->getTrafos(GeometricType::Affinity); */
    /* std::cout << matches.size() << std::endl; */

    // do pano stitching for every wanted type
    /* auto stitcher = std::make_unique<PanoramaSticher>(imgContainer, ftContainer, matchContainer);
     */
    /* stitcher->initTrafos(); */
    /* auto panoImg0 = sticher->stitchPano(); */

    /* sticher->globalOptimize(); */
    /* auto panoImg1 = sticher->stitchPano(); */
    /* auto trafoList = sticher->getTrafoList(); */

    // keep keyframe transformation constant
    // init non-keyframes by interpolating
    // bundleadjust
    /* sticher->reintegrate(); */

    return 0;
}
