#ifndef HABITRACK_PANORAMA_STITCHER_H
#define HABITRACK_PANORAMA_STITCHER_H

#include "habitrack/baseFeatureContainer.h"
#include "habitrack/baseImageContainer.h"

#include "habitrack/geometricType.h"
#include "habitrack/matchesContainer.h"
#include "habitrack/transformation.h"

#include <opencv2/core.hpp>

#include <unordered_set>

namespace ceres
{
class Problem;
}

namespace ht
{
enum class Blending : bool
{
    NoBlend,
    Blend
};

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
    PanoramaStitcher(std::shared_ptr<BaseImageContainer> imgContainer,
        std::shared_ptr<BaseFeatureContainer> ftContainer, const PairwiseMatches& matches,
        const PairwiseTrafos& trafos, const std::vector<size_t>& keyFrames, GeometricType type,
        Blending blend, const cv::Mat& camMat = cv::Mat(), const cv::Mat& distCoeffs = cv::Mat());

    PanoramaStitcher(std::shared_ptr<BaseImageContainer> imgContainer,
        std::shared_ptr<BaseFeatureContainer> ftContainer,
        std::shared_ptr<MatchesContainer> matchContainer, const std::vector<size_t>& keyFrames,
        GeometricType type, Blending blend, const cv::Mat& camMat = cv::Mat(),
        const cv::Mat& distCoeffs = cv::Mat());

    void initTrafos();
    void initTrafosFromMultipleVideos(const std::vector<std::size_t> sizes,
        const std::vector<std::vector<cv::Mat>>& localOptimalTrafos,
        const std::unordered_map<std::pair<std::size_t, std::size_t>,
            std::pair<std::size_t, std::size_t>>& optimalTransitions);
    void initTrafosMultipleHelper(std::size_t currBlock, const cv::Mat& currTrafo,
        const std::vector<cv::Mat>& localOptimalTrafos, const std::vector<std::size_t>& sizes);
    std::tuple<cv::Mat, cv::Mat, cv::Mat> stitchPano(cv::Size targetSize, bool drawCenters = false);

    void globalOptimizeKeyFrames(std::size_t limitTo = 0);
    void refineNonKeyFrames(const PairwiseMatches& matches, std::size_t limitTo = 0);
    void reintegrate();

    static std::vector<cv::Mat> loadTrafos(const std::filesystem::path& file);
    void writeTrafos(const std::filesystem::path& file, WriteType writeType = WriteType::Binary);

private:
    void highlightImg(cv::Mat& img);
    std::vector<std::size_t> sortIdsByResponseProduct(
        const std::vector<cv::KeyPoint>& ftsI, const std::vector<cv::KeyPoint>& ftsJ);
    std::vector<cv::KeyPoint> permute(
        const std::vector<cv::KeyPoint>& fts, const std::vector<std::size_t>& p);
    void globalOptimizeHelper(
        const PairwiseMatches& matches, FramesMode keyFramesMode, std::size_t limitTo);
    cv::Rect2d generateBoundingRect() const;
    cv::Rect2d generateBoundingRectHelper(
        const cv::Mat& trafo, cv::Rect2d currRect = cv::Rect2d()) const;
    cv::Mat draw(const cv::Mat& trafo, std::size_t idI, std::size_t idJ);

    void addFunctor(ceres::Problem& problem, const cv::Point2f& ptI, const cv::Point2f& ptJ,
        cv::Mat* trafoI, cv::Mat* trafoJ, double* camParams, double* distParams,
        std::vector<double>* paramsI, std::vector<double>* paramsJ, double weight = 1.0);
    void repairTrafos(const std::vector<std::vector<double>>& params, FramesMode framesMode);

    std::vector<double> getCamParameterization() const;
    std::vector<double> getDistParameterization() const;
    std::vector<double> invertDistParameterization(const std::vector<double>& params) const;
    void repairIntriniscs(
        const std::vector<double>& camParams, const std::vector<double>& distParams);

    std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>> getCorrespondingPoints(
        std::pair<std::size_t, std::size_t> pair, const Matches& matches);
    cv::Point getCenter(const cv::Mat& trafo);

    cv::Mat transformBoundingRect(const cv::Mat& trafo) const;
    cv::Mat interpolateTrafo(double alpha, const cv::Mat& mat1, const cv::Mat& mat2) const;

    inline bool isKeyFrame(std::size_t idx) { return mKeyFramesSet.count(idx); };

private:
    std::shared_ptr<BaseImageContainer> mImgContainer;
    std::shared_ptr<BaseFeatureContainer> mFtContainer;
    PairwiseMatches mMatches;
    PairwiseTrafos mTrafos;
    std::vector<std::size_t> mKeyFrames;
    std::unordered_set<std::size_t> mKeyFramesSet;
    GeometricType mType;
    bool mBlend;

    cv::Mat mCamMat;
    cv::Mat mCamMatInv;
    cv::Mat mDistCoeffs;

    std::vector<cv::Mat> mOptimizedTrafos;
};
} // namespace ht
#endif // HABITRACK_PANORAMA_STITCHER_H
