#include "resultsIO.h"

#include <opencv2/core/persistence.hpp>
#include "spdlog/spdlog.h"

namespace gui
{
namespace fs = std::filesystem;
namespace detail
{
    void savePreferences(cv::FileStorage& fs, const Preferences& prefs)
    {
        fs << "preferences"
           << "{";
        // general
        fs << "cache_size" << prefs.cacheSize;
        fs << "chunk_size" << prefs.chunkSize;

        // colour correction
        fs << "colour_correction" << prefs.colourCorrection;
        fs << "colour_red" << prefs.colourRed;
        fs << "colour_green" << prefs.colourGreen;
        fs << "colour_blue" << prefs.colourBlue;

        // features
        fs << "feature_type" << prefs.featureType;
        fs << "num_features" << prefs.numFeatures;

        // unaries
        fs << "unary_subsample" << prefs.unarySubsample;
        fs << "unary_sigma" << prefs.unarySigma;
        fs << "unary_remove_lasers" << prefs.removeRedLasers;
        fs << "unary_suppress" << prefs.unarySuppress;
        fs << "unary_multiplier" << prefs.unaryMultiplier;

        // pairwise
        fs << "pairwise_size" << prefs.pairwiseSize;
        fs << "pairwise_sigma" << prefs.pairwiseSigma;

        // smooth bearing
        fs << "smooth_bearing" << prefs.smoothBearing;
        fs << "smooth_bearing_window_size" << prefs.smoothBearingWindowSize;
        fs << "smooth_bearing_outlier_tolerance" << prefs.smoothBearingOutlierTol;

        // transformation
        fs << "remove_camera_motion" << prefs.removeCamMotion;
        fs << "nn_ratio" << prefs.nnRatio;
        fs << "ransac_reproj" << prefs.ranscacReproj;
        fs << "}";
    }

    Preferences loadPreferences(const cv::FileStorage& fs)
    {
        auto node = fs["preferences"];
        Preferences prefs;

        // general
        prefs.cacheSize = node["cache_size"];
        prefs.chunkSize = node["cache_size"];

        // colour correction
        prefs.colourCorrection = static_cast<bool>(static_cast<int>(node["cache_size"]));
        prefs.colourRed = node["cache_size"];
        prefs.colourGreen = node["cache_size"];
        prefs.colourBlue = node["cache_size"];

        // features
        prefs.featureType = static_cast<ht::FeatureType>(static_cast<int>(node["cache_size"]));
        prefs.numFeatures = node["cache_size"];

        // unaries, unary recompute needed
        prefs.unarySubsample = node["cache_size"];
        prefs.unarySigma = node["cache_size"];
        prefs.removeRedLasers = static_cast<bool>(static_cast<int>(node["cache_size"]));
        prefs.unarySuppress = node["cache_size"];
        prefs.unaryMultiplier = node["cache_size"];

        // pairwise, tracking recompute needed
        prefs.pairwiseSize = node["cache_size"];
        prefs.pairwiseSigma = node["cache_size"];

        // smooth bearing, tracking recompute needed
        prefs.smoothBearing = static_cast<bool>(static_cast<int>(node["cache_size"]));
        prefs.smoothBearingWindowSize = node["cache_size"];
        prefs.smoothBearingOutlierTol = node["cache_size"];

        // transformation
        prefs.removeCamMotion = static_cast<bool>(static_cast<int>(node["cache_size"]));
        prefs.nnRatio = node["cache_size"];
        prefs.ranscacReproj = node["cache_size"];

        return prefs;
    }
} // namespace detail

void saveResults(const fs::path& resultFile, const Preferences& prefs, const fs::path& imgFolder,
    std::size_t startFrame, std::size_t endFrame)
{
    spdlog::info("Save results file to: {}", resultFile.string());
    cv::FileStorage fs(resultFile.string(), cv::FileStorage::WRITE);

    fs << "img_folder" << imgFolder.string();
    fs << "start_frame" << static_cast<int>(startFrame);
    fs << "end_frame" << static_cast<int>(endFrame);
    detail::savePreferences(fs, prefs);
}

std::tuple<Preferences, std::filesystem::path, std::size_t, std::size_t> loadResults(
    const std::filesystem::path& resultFile)
{
    spdlog::info("Save results file to: {}", resultFile.string());
    cv::FileStorage fs(resultFile.string(), cv::FileStorage::READ);

    auto imgFolder = static_cast<fs::path>(fs["img_folder"]);
    auto startFrame = static_cast<std::size_t>(static_cast<int>(fs["start_frame"]));
    auto endFrame = static_cast<std::size_t>(static_cast<int>(fs["end_frame"]));
    auto prefs = detail::loadPreferences(fs);
    return {prefs, imgFolder, startFrame, endFrame};
}

} // namespace gui
