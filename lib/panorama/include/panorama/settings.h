#ifndef HABITRACK_SETTINGS_H
#define HABITRACK_SETTINGS_H

#include <ostream>
#include "image-processing/features.h"

namespace ht
{
enum class PanoramaStage
{
    Initialization,
    Optimization,
    Refinement
};

std::ostream& operator<<(std::ostream& stream, const PanoramaStage& stage);

struct PanoramaSettings
{
    int rows = 8000;
    int cols = 8000;
    int cacheSize = 200;

    FeatureType ftType = FeatureType::ORB;
    int numFts = 500;
    double minCoverage = 0.0;
    bool force = false;
    PanoramaStage stage = PanoramaStage::Refinement;

    bool overlayCenters = true;
    bool overlayPoints = false;
    bool smooth = false;

    bool writeReadable = true;
};
std::ostream& operator<<(std::ostream& stream, const PanoramaSettings& prefs);

struct GPSSettings
{
    std::filesystem::path file;
    double offset;
    int sampling_rate;
    int frame_sampling_rate;

    bool has_prior() const { return !file.empty(); }
};
std::ostream& operator<<(std::ostream& stream, const GPSSettings& gps);

} // ht
#endif // HABITRACK_SETTINGS_H
