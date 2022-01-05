#ifndef HABITRACK_PANORAMA_ENGINE_H
#define HABITRACK_PANORAMA_ENGINE_H

#include "panorama/gpsInterpolator.h"

namespace ht
{
namespace PanoramaEngine
{
    GPSSettings loadGPSSettings(const std::filesystem::path& gpsFile);

    void runSingle(const Images& images, const std::filesystem::path& dataFolder,
        const PanoramaSettings& settings, const std::vector<cv::Point>& overlayPts = {},
        std::size_t chunkSize = 0, const GPSInterpolator& gps = {},
        std::shared_ptr<BaseProgressBar> mBar = {});

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
