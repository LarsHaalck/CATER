#ifndef HABITRACK_PANORAMA_ENGINE_H
#define HABITRACK_PANORAMA_ENGINE_H

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
    int rows = 2000;
    int cols = 2000;
    int cacheSize = 200;

    FeatureType ftType = FeatureType::ORB;
    int numFts = 500;
    double minCoverage = 0.0;
    bool force = false;
    PanoramaStage stage = PanoramaStage::Refinement;

    bool overlayCenters = false;
    bool overlayPoints = false;
    bool smooth = false;

    bool writeReadable = false;
};
std::ostream& operator<<(std::ostream& stream, const PanoramaSettings& prefs);

namespace PanoramaEngine
{
    void runSingle(const Images& images, const std::filesystem::path& dataFolder,
        const PanoramaSettings& settings, const std::vector<cv::Point>& overlayPts = {},
        std::size_t chunkSize = 0, std::shared_ptr<BaseProgressBar> mBar = {});

    void runMulti(const std::vector<Images>& images,
        const std::vector<std::filesystem::path>& dataFolders,
        const std::filesystem::path& outFolder, const PanoramaSettings& settings,
        const std::vector<cv::Point>& overlayPts = {},
        const std::vector<std::size_t>& chunkSizes = {},
        std::shared_ptr<BaseProgressBar> mBar = {});

    namespace detail
    {
        void overlay(const cv::Mat& img, const std::vector<cv::Point>& pts,
            const std::vector<cv::Mat>& trafos, const std::filesystem::path& filename,
            const PanoramaSettings& settings, cv::Point2d imgCenter,
            const std::vector<std::size_t>& chunkSizes, const std::vector<std::size_t>& sizes);

        std::vector<cv::Point> smooth(const std::vector<cv::Point>& pts,
            const std::vector<std::size_t>& chunkSizes, const std::vector<std::size_t>& sizes);

    } // namespace detail
} // namespace PanoramaEngine
} // namespace ht

#endif // HABITRACK_PANORAMA_ENGINE_H
