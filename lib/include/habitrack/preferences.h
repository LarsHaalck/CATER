#ifndef HABITRACK_PREFERENCES_H
#define HABITRACK_PREFERENCES_H

/* #include "misc/geometricType.h" */

#include <cstddef>

namespace ht
{
/* struct FeaturePreferences */
/* { */
/*     enum class Type */
/*     { */
/*         ORB, */
/*         SIFT */
/*     }; */
/*     Type type = Type::ORB; */
/*     int numFeatures = 10000; */
/* }; */

/* struct CachePreferences */
/* { */
/*     bool parallelReadWrite = true; // should only be used for ssds */
/*     std::size_t cacheSize = 400; */
/* }; */

struct Preferences
{
    // cache related
    /* bool imageMemOnly = false; */

    /* // warping */
    /* bool removeCameraMotion; */
    /* double warpingNNRatio; */
    /* double warpingRANSACReprojError; */
    /* int warpingNumFeatures; */

    /* // unaries */
    /* double unariesSubsample; */
    /* float unariesSigma; */
    /* bool unariesRemoveRedLasers; */
    /* double unariesSupression; */
    /* double manualTrackingUnaryMultiplier; */

    /* // pairwise */
    /* double pairwiseSigma; */
    /* int pairwiseSize; */

    /* // color adjusting */
    /* double blueFactor; */
    /* double greenFactor; */
    /* double redFactor; */

    /* // bearings */
    /* bool smoothBearing; */
    /* int smoothBearingWindowSize; */
    /* int smoothBearingOutlierTolerance; */

    /* // tracklets */
    /* int trackletsLength; */

    /* // panoramos */
    /* bool generatePano; */
    /* int panoWindowSize; */
    /* GeometricType panoTransformation; */
    /* double panoMinOffset; */
    /* double panoMaxOffset; */
    /* int panoTargetRows; */
    /* int panoTargetCols; */
    /* bool panoEnableBlending; */
};
} // namespace ht
#endif // HABITRACK_PREFERENCES_H
