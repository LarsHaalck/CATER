#include "resultsIO.h"

#include <opencv2/core/persistence.hpp>
#include "spdlog/spdlog.h"

namespace gui
{
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
void saveResults(const std::filesystem::path& resultFile, const Preferences& prefs)
{

    spdlog::debug("Save results file to: {}", resultFile.string());
    cv::FileStorage fs(resultFile.string(), cv::FileStorage::WRITE);
    detail::savePreferences(fs, prefs);

    /*fs << "video_path" << QtOpencvCore::qstr2str(trackingData.getVideoPath());*/
    /*fs << "abs_path" << QtOpencvCore::qstr2str(trackingData.getAbsPath());*/
    /*fs << "file_name" << QtOpencvCore::qstr2str(trackingData.getFileName());*/
    /*fs << "file_format" << QtOpencvCore::qstr2str(trackingData.getFileFormat());*/
    /*fs << "image_folder_used" << trackingData.getIsImageFolderUsed();*/

    /*fs << "abs_output_path"*/
    /*   << QtOpencvCore::qstr2str(trackingData.getAbsOutputPath());*/
    /*fs << "results_output_path"*/
    /*   << QtOpencvCore::qstr2str(trackingData.getResultsOutputPath());*/

    /*fs << "date" << QtOpencvCore::qstr2str(trackingData.getDate());*/
    /*fs << "time" << QtOpencvCore::qstr2str(trackingData.getTime());*/

    /*int numOfFrames = trackingData.getNumberOfFrames();*/
    /*fs << "num_of_frames" << numOfFrames;*/
    /*int firstFrameUsed = trackingData.getFirstFrameUsedNum();*/
    /*fs << "first_used_frame_num" << firstFrameUsed;*/
    /*int lastFrameUsed = trackingData.getLastFrameUsedNum();*/
    /*fs << "last_used_frame_num" << lastFrameUsed;*/
    /*int numOfFramesUsed = lastFrameUsed - firstFrameUsed;*/
    /*fs << "num_of_frames_used" << numOfFramesUsed;*/

    /*fs << "has_features" << trackingData.hasFeaturesExtracted();*/
    /*fs << "features_path"*/
    /*   << QtOpencvCore::qstr2str(trackingData.getFeaturesOutputPath());*/

    /*fs << "has_transformations" << trackingData.hasTransformationsCalculated();*/
    /*fs << "transformations_path"*/
    /*   << QtOpencvCore::qstr2str(trackingData.getTransformationsOutputPath());*/

    /*fs << "has_homographies" << trackingData.hasHomographiesCalculated();*/
    /*fs << "homography_file"*/
    /*   << QtOpencvCore::qstr2str(trackingData.getHomographiesOutputPath());*/

    /*fs << "has_unaries" << trackingData.hasUnariesCalculated();*/
    /*fs << "unaries_path"*/
    /*   << QtOpencvCore::qstr2str(trackingData.getUnariesFolderPath());*/
    /*fs << "unaries_file"*/
    /*   << QtOpencvCore::qstr2str(trackingData.getUnariesOutputPath());*/
    /*fs << "overall_gaussian_mask"*/
    /*   << QtOpencvCore::qstr2str(trackingData.getOverallGaussianMaskOutputPath());*/
    /*fs << "has_ant_positions" << trackingData.hasAntPositionsCalculated();*/
    /*fs << "ant_position_path"*/
    /*   << QtOpencvCore::qstr2str(trackingData.getAntOutputPath());*/

    /*// save preferences*/
    /*fs << "preferences"*/
    /*   << "{";*/
    /*fs << "center_x" << trackingData.getCenterX();*/
    /*fs << "center_y" << trackingData.getCenterY();*/
    /*fs << "num_rows" << trackingData.getNumRows();*/
    /*fs << "num_cols" << trackingData.getNumCols();*/

    /*// save used frame numbers*/
    /*std::vector<int> frameNumbers = trackingData.getFrameNumberVector();*/
    /*fs << "frame_no"*/
    /*   << "[";*/
    /*for (auto it = frameNumbers.begin(); it != frameNumbers.end(); ++it)*/
    /*{*/
    /*    fs << *it;*/
    /*}*/
    /*fs << "]";*/
}

} // namespace gui
