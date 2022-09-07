#ifndef CATER_SETTINGS_H
#define CATER_SETTINGS_H

#include <ostream>
#include <cater/image-processing/features.h>

namespace ct
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
    int rows = 2000;
    int cols = 2000;
    int cacheSize = 200;

    FeatureType featureType = FeatureType::ORB;
    int numFeatures = 500;
    double minCoverage = 0.0;
    bool force = false;
    PanoramaStage stage = PanoramaStage::Refinement;

    bool overlayCenters = false;
    bool overlayPoints = false;
    bool smooth = false;

    bool writeReadable = false;
};
std::ostream& operator<<(std::ostream& stream, const PanoramaSettings& prefs);

struct GPSSettings
{
    std::filesystem::path file;
    double offset;
    double sampling_rate;
    double frame_sampling_rate;

    bool has_prior() const { return !file.empty(); }
};
std::ostream& operator<<(std::ostream& stream, const GPSSettings& gps);

} // ht
#endif // CATER_SETTINGS_H
