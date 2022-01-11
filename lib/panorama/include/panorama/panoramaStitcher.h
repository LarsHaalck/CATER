#ifndef HABITRACK_PANORAMA_STITCHER_H
#define HABITRACK_PANORAMA_STITCHER_H

#include "image-processing/baseFeatureContainer.h"
#include "image-processing/baseImageContainer.h"
#include "image-processing/geometricType.h"
#include "image-processing/matches.h"
#include "image-processing/transformation.h"
#include "panorama/gpsInterpolator.h"
#include "util/pairHash.h"

#include <opencv2/core.hpp>
#include <unordered_set>

namespace ceres
{
class Problem;
}

namespace cv::detail
{
class GainCompensator;
}

namespace ht
{
enum class FramesMode : bool
{
    KeyFramesOnly,
    AllFrames
};

enum class WriteType : bool
{
    Binary,
    Readable
};
} // namespace ht

namespace ht
{
class PanoramaStitcher
{
public:
    PanoramaStitcher(const BaseImageContainer& images, const std::vector<size_t>& keyFrames,
        GeometricType type, const cv::Mat& camMat = cv::Mat(),
        const cv::Mat& distCoeffs = cv::Mat());

    void initTrafos(const PairwiseTrafos& trafos);
    void initTrafosFromMultipleVideos(const PairwiseTrafos& trafos,
        const std::vector<std::size_t> sizes,
        const std::vector<std::vector<cv::Mat>>& localOptimalTrafos,
        const std::unordered_map<std::pair<std::size_t, std::size_t>,
            std::pair<std::size_t, std::size_t>, hash>& optimalTransitions);
    void initTrafosMultipleHelper(std::size_t currBlock, const cv::Mat& currTrafo,
        const std::vector<cv::Mat>& localOptimalTrafos, const std::vector<std::size_t>& sizes);

    std::tuple<cv::Mat, std::vector<cv::Mat>> stitchPano(
        cv::Size targetSize, bool blend = false, std::shared_ptr<BaseProgressBar> cb = {}) const;

    void globalOptimizeKeyFrames(const BaseFeatureContainer& fts, const PairwiseMatches& matches,
        std::size_t limitTo = 0, const GPSMap& gps = {}, bool fixTranslation = false,
        std::shared_ptr<BaseProgressBar> cb = {});
    void refineNonKeyFrames(const BaseFeatureContainer& fts, const PairwiseMatches& matches,
        std::size_t limitTo = 0, std::shared_ptr<BaseProgressBar> cb = {});
    void reintegrate();

    void loadTrafos(const std::filesystem::path& file);
    static std::vector<cv::Mat> getTrafos(const std::filesystem::path& file);

    void writeTrafos(const std::filesystem::path& file, WriteType writeType = WriteType::Binary);

private:
    void buildParamsVector();
    std::vector<std::size_t> sortIdsByResponseProduct(const std::vector<cv::KeyPoint>& ftsI,
        const std::vector<cv::KeyPoint>& ftsJ, const std::vector<float>& weights);

    bool globalOptimize(const BaseFeatureContainer& fts, const PairwiseMatches& matches,
        FramesMode keyFramesMode, std::size_t limitTo, bool multiThread, const GPSMap& gps = {},
        bool fixTranslation = false, std::shared_ptr<BaseProgressBar> cb = {});

    std::tuple<cv::Mat, cv::Mat, cv::Size> scaleTransMat(cv::Size targetSize) const;
    cv::Rect2d generateBoundingRect() const;
    cv::Rect2d generateBoundingRectHelper(
        const cv::Mat& trafo, cv::Rect2d currRect = cv::Rect2d()) const;

    void addFunctor(ceres::Problem& problem, const cv::Point2f& ptI, const cv::Point2f& ptJ,
        cv::Mat* trafoI, cv::Mat* trafoJ, double* camParams, double* distParams,
        std::vector<double>* paramsI, std::vector<double>* paramsJ, float response, float weight);
    void addGPSRegularizer(ceres::Problem& problem, const GPSMap& gps);
    void fixGPSTranslation(ceres::Problem& problem, const GPSMap& gps);
    void reconstructTrafos(FramesMode framesMode);

    std::vector<double> getCamParameterization() const;
    std::vector<double> getDistParameterization() const;
    std::vector<double> invertDistParameterization(const std::vector<double>& params) const;
    void repairIntriniscs(
        const std::vector<double>& camParams, const std::vector<double>& distParams);

    std::tuple<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>, std::vector<float>>
    getCorrespondingPoints(std::pair<std::size_t, std::size_t> pair, const Matches& matches,
        const BaseFeatureContainer& fts);

    cv::Mat transformBoundingRect(const cv::Mat& trafo) const;
    cv::Mat interpolateTrafo(double alpha, const cv::Mat& mat1, const cv::Mat& mat2) const;

    inline bool isKeyFrame(std::size_t idx) { return mKeyFramesSet.count(idx); };

    void prepareCompensator(cv::detail::GainCompensator& c) const;

private:
    const BaseImageContainer& mImages;
    std::vector<std::size_t> mKeyFrames;
    std::unordered_set<std::size_t> mKeyFramesSet;
    GeometricType mType;

    cv::Mat mCamMat;
    cv::Mat mCamMatInv;
    cv::Mat mDistCoeffs;

    std::vector<cv::Mat> mOptimizedTrafos;
    std::vector<std::vector<double>> mOptimizedParams; // use for isometries only

    bool mReintegrated;

    cv::Mat mGPSSim;
};
} // namespace ht
#endif // HABITRACK_PANORAMA_STITCHER_H
