#ifndef MODEL_PREFERENCES_H
#define MODEL_PREFERENCES_H

#include <cater/image-processing/baseFeatureContainer.h>
#include <ostream>

namespace ct
{
struct Preferences
{
    // general
    int cacheSize = 400;
    int chunkSize = 100; // tracking recompute needed
    int detectionRadius = 20;
    int fps = 0;
    int pixelsPerMm = 0;

    // features
    ct::FeatureType featureType = ct::FeatureType::ORB;
    int numFeatures = 500;

    // unaries, unary recompute needed
    double unarySubsample = 0.8;
    double unarySigma = 200.0;
    bool removeRedLasers = true;
    double unarySuppress = 1.0;
    double unaryMultiplier = 4.0;
    int manualUnarySize = 9;

    // only set as a debug option by TUI
    bool ignoreManualUnaries = false;

    // pairwise, tracking recompute needed
    int pairwiseSize = 25;
    double pairwiseSigma = 6.0;

    // smooth bearing, tracking recompute needed
    bool smoothBearing = true;
    int smoothBearingWindowSize = 5;
    int smoothBearingOutlierTol = 3;

    // transformation
    bool removeCamMotion = true;
    double nnRatio = 0.8;
    double ranscacReproj = 3.0;
};
std::ostream& operator<<(std::ostream& stream, const Preferences& prefs);
} // namespace ct

#endif // MODEL_PREFERENCES_H
