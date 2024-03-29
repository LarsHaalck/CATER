#include <cater/model/resultsIO.h>

#include <cater/version/version.h>

#include <spdlog/spdlog.h>
#include <opencv2/core/persistence.hpp>


namespace ct
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
        fs << "detection_radius" << prefs.detectionRadius;
        fs << "fps" << prefs.fps;
        fs << "pixels_per_mm" << prefs.pixelsPerMm;

        // features
        fs << "feature_type" << prefs.featureType;
        fs << "num_features" << prefs.numFeatures;

        // unaries
        fs << "unary_subsample" << prefs.unarySubsample;
        fs << "unary_sigma" << prefs.unarySigma;
        fs << "unary_remove_lasers" << prefs.removeRedLasers;
        fs << "unary_suppress" << prefs.unarySuppress;
        fs << "unary_multiplier" << prefs.unaryMultiplier;
        fs << "manual_unary_size" << prefs.manualUnarySize;

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
        prefs.chunkSize = node["chunk_size"];

        if (!node["detection_radius"].empty())
            prefs.detectionRadius = node["detection_radius"];
        if (!node["fps"].empty())
            prefs.fps = node["fps"];
        if (!node["pixels_per_mm"].empty())
            prefs.pixelsPerMm = node["pixels_per_mm"];

        // features
        prefs.featureType = static_cast<ct::FeatureType>(static_cast<int>(node["feature_type"]));
        prefs.numFeatures = node["num_features"];

        // unaries, unary recompute needed
        prefs.unarySubsample = node["unary_subsample"];
        prefs.unarySigma = node["unary_sigma"];
        prefs.removeRedLasers = static_cast<bool>(static_cast<int>(node["unary_remove_lasers"]));
        prefs.unarySuppress = node["unary_suppress"];
        prefs.unaryMultiplier = node["unary_multiplier"];

        if (!node["manual_unary_size"].empty())
            prefs.manualUnarySize = node["manual_unary_size"];

        // pairwise, tracking recompute needed
        prefs.pairwiseSize = node["pairwise_size"];
        prefs.pairwiseSigma = node["pairwise_sigma"];

        // smooth bearing, tracking recompute needed
        prefs.smoothBearing = static_cast<bool>(static_cast<int>(node["smooth_bearing"]));
        prefs.smoothBearingWindowSize = node["smooth_bearing_window_size"];
        prefs.smoothBearingOutlierTol = node["smooth_bearing_outlier_tolerance"];

        // transformation
        prefs.removeCamMotion = static_cast<bool>(static_cast<int>(node["remove_camera_motion"]));
        prefs.nnRatio = node["nn_ratio"];
        prefs.ranscacReproj = node["ransac_reproj"];

        return prefs;
    }
} // namespace detail

void saveResults(const fs::path& resultFile, const Preferences& prefs, const fs::path& imgFolder,
    std::size_t startFrame, std::size_t endFrame)
{
    spdlog::info("Save results file to: {}", resultFile.string());
    cv::FileStorage fs(resultFile.string(), cv::FileStorage::WRITE);

    fs << "version" << CATER_VER;
    fs << "img_folder" << imgFolder.string();
    fs << "start_frame" << static_cast<int>(startFrame);
    fs << "end_frame" << static_cast<int>(endFrame);
    detail::savePreferences(fs, prefs);
}

std::tuple<Preferences, std::filesystem::path, std::size_t, std::size_t> loadResults(
    const std::filesystem::path& resultFile)
{
    spdlog::info("load results from: {}", resultFile.string());
    cv::FileStorage fs(resultFile.string(), cv::FileStorage::READ);

    auto imgFolder = static_cast<fs::path>(fs["img_folder"]);
    auto startFrame = static_cast<std::size_t>(static_cast<int>(fs["start_frame"]));
    auto endFrame = static_cast<std::size_t>(static_cast<int>(fs["end_frame"]));
    auto version = static_cast<std::string>(fs["version"]);

    // TODO: check for non existent version or smaller version and do warning warning
    auto prefs = detail::loadPreferences(fs);

    spdlog::debug("Img folder: {}", imgFolder.string());
    spdlog::debug("start frame: {}", startFrame);
    spdlog::debug("end frame: {}", endFrame);
    spdlog::debug("version: {}", version);
    return {prefs, imgFolder, startFrame, endFrame};
}

} // namespace ct
