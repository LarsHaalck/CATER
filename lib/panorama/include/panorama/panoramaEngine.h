#ifndef HABITRACK_PANORAMA_ENGINE_H
#define HABITRACK_PANORAMA_ENGINE_H

#include "image-processing/features.h"

namespace ht
{
struct PanoramaSettings
{
    bool force;
    int stage;
    int cacheSize;
    int rows;
    int cols;
    int numFts;
    FeatureType ftType;

    bool writeReadable = false;
};

namespace PanoramaEngine
{
    void runSingle(const Images& images, const std::filesystem::path& dataFolder,
        const PanoramaSettings& settings, std::shared_ptr<BaseProgressBar> mBar = {});

    void runMulti(const std::vector<Images>& images,
        const std::vector<std::filesystem::path>& dataFolders,
        const std::filesystem::path& outFolder, const PanoramaSettings& settings,
        std::shared_ptr<BaseProgressBar> mBar = {});
} // namespace PanoramaEngine
} // namespace ht

#endif // HABITRACK_PANORAMA_ENGINE_H
