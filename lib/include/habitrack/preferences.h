#ifndef PREFERENCESSTRUCT
#define PREFERENCESSTRUCT

#include "misc/geometricType.h"

namespace at
{
struct Preferences
{
    // warping
    bool removeCameraMotion;
    double warpingNNRatio;
    double warpingRANSACReprojError;
    int warpingNumFeatures;

    // unaries
    double unariesSubsample;
    float unariesSigma;
    bool unariesRemoveRedLasers;
    double unariesSupression;
    double manualTrackingUnaryMultiplier;

    // pairwise
    double pairwiseSigma;
    int pairwiseSize;

    // color adjusting
    double blueFactor;
    double greenFactor;
    double redFactor;

    // bearings
    bool smoothBearing;
    int smoothBearingWindowSize;
    int smoothBearingOutlierTolerance;

    // tracklets
    int trackletsLength;

    // panoramos
    bool generatePano;
    int panoWindowSize;
    GeometricType panoTransformation;
    double panoMinOffset;
    double panoMaxOffset;
    int panoTargetRows;
    int panoTargetCols;
    bool panoEnableBlending;
};
} // namespace at
#endif // PREFERENCESSTRUCT
