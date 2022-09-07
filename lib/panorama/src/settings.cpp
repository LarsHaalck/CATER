#include <cater/panorama/settings.h>

namespace ct
{
std::ostream& operator<<(std::ostream& stream, const PanoramaStage& stage)
{
    switch (stage)
    {
    case PanoramaStage::Initialization:
        stream << "Initialization\n";
        break;
    case PanoramaStage::Optimization:
        stream << "Optimization\n";
        break;
    case PanoramaStage::Refinement:
        stream << "Refinement\n";
        break;
    default:
        stream << "Undefined\n";
        break;
    }
    return stream;
}

std::ostream& operator<<(std::ostream& stream, const PanoramaSettings& settings)
{
    stream << "Panorama Settings: \n";
    stream << "General: \n";
    stream << "rows: " << settings.rows << "\n";
    stream << "cols: " << settings.cols << "\n";
    stream << "cacheSize: " << settings.cacheSize << "\n";
    stream << "--------------------\n";
    stream << "ftType: " << settings.featureType << "\n";
    stream << "numFts: " << settings.numFeatures << "\n";
    stream << "minCoverage: " << settings.minCoverage << "\n";
    stream << "force: " << settings.force << "\n";
    stream << "stage: " << settings.stage << "\n";
    stream << "--------------------\n";
    stream << "overlayCenters: " << settings.overlayCenters << "\n";
    stream << "overlayPoints: " << settings.overlayPoints << "\n";
    stream << "smooth: " << settings.smooth << "\n";
    stream << "--------------------\n";
    stream << "writeReadable: " << settings.writeReadable << "\n";
    stream << "====================\n";
    return stream;
}

std::ostream& operator<<(std::ostream& stream, const GPSSettings& gps)
{
    stream << "GPS: \n";
    stream << "file: " << gps.file << "\n";
    stream << "offset: " << gps.offset << "\n";
    stream << "gps sampling rate: " << gps.sampling_rate << "\n";
    stream << "frame sampling rate: " << gps.frame_sampling_rate << "\n";
    stream << "====================\n";
    return stream;
}
} // ht
