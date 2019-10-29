#ifndef HABITRACK_PANORAMA_STITCHER_H
#define HABITRACK_PANORAMA_STITCHER_H

#include "habitrack/featureContainer.h"
#include "habitrack/geometricType.h"
#include "habitrack/imageContainer.h"
#include "habitrack/matchesContainer.h"
#include "habitrack/transformation.h"

#include <opencv2/core.hpp>

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
} // namespace ht

namespace ht
{
class PanoramaStitcher
{
public:
    PanoramaStitcher(std::shared_ptr<ImageContainer> imgContainer,
        std::shared_ptr<FeatureContainer> ftContainer,
        std::shared_ptr<MatchesContainer> matchContainer, const std::vector<size_t>& keyFrames,
        GeometricType type, Blending blend, const cv::Mat& camMat = cv::Mat(),
        const cv::Mat& distCoeffs = cv::Mat());

    void initTrafos(GeometricType type = GeometricType::Undefined);
    std::tuple<cv::Mat, cv::Mat, cv::Mat> stitchPano(cv::Size targetSize, bool drawCenters = false);
    void globalOptimize();
    void reintegrate();
private:
    cv::Rect2d generateBoundingRect() const;
    cv::Rect2d generateBoundingRectHelper(
        const cv::Mat& trafo, cv::Rect2d currRect = cv::Rect2d()) const;
    cv::Mat draw(const cv::Mat& trafo, size_t idI, size_t idJ);

    void addFunctor(ceres::Problem& problem, const cv::Point2f& ptI, const cv::Point2f& ptJ,
        cv::Mat* trafoI, cv::Mat* trafoJ, double* camParams, double* distParams,
        std::vector<double>* paramsI, std::vector<double>* paramsJ, double weight = 1.0);
    void repairTrafos(const std::vector<std::vector<double>>& params);

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

private:
    std::shared_ptr<ImageContainer> mImgContainer;
    std::shared_ptr<FeatureContainer> mFtContainer;
    std::shared_ptr<MatchesContainer> mMatchContainer;
    PairwiseMatches mMatches;
    PairwiseTrafos mTrafos;
    std::vector<size_t> mKeyFrames;
    GeometricType mType;
    bool mBlend;

    cv::Mat mCamMat;
    cv::Mat mCamMatInv;
    cv::Mat mDistCoeffs;

    std::vector<cv::Mat> mOptimizedTrafos;
};
} // namespace ht
#endif // HABITRACK_PANORAMA_STITCHER_H
