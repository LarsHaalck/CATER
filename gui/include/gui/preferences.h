#ifndef GUI_PREFERENCES_H
#define GUI_PREFERENCES_H

#include "image-processing/baseFeatureContainer.h"
#include "spdlog/fmt/ostr.h"
#include <ostream>

namespace gui
{
struct Preferences
{
    // general
    int cacheSize = 400;
    int chunkSize = 100;

    // colour correction TODO: needed?
    bool colourCorrection = false;
    int colourRed = 33;
    int colourGreen = 33;
    int colourBlue = 33;

    // features
    ht::FeatureType featureType = ht::FeatureType::ORB;
    int numFeatures = 500;

    // unaries, unary recompute needed
    double unarySubsample = 0.8;
    double unarySigma = 200.0;
    bool removeRedLasers = true;
    double unarySuppress = 1.0;
    double unaryMultiplier = 4.0;
    int manualUnarySize = 9;

    // pairwise, tracking recompute needed
    int pairwiseSize = 15;
    double pairwiseSigma = 6.0;

    // smooth bearing, tracking recompute needed
    bool smoothBearing = true;
    int smoothBearingWindowSize = 5;
    int smoothBearingOutlierTol = 3;

    // transformation
    bool removeCamMotion = true;
    double nnRatio = 0.8;
    double ranscacReproj = 3.0;

    friend std::ostream& operator<<(std::ostream& stream, const gui::Preferences& prefs);
};
} // namespace gui

#endif // GUI_PREFERENCES_H
