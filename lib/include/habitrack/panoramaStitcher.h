#ifndef HABITRACK_PANORAMA_STITCHER_H
#define HABITRACK_PANORAMA_STITCHER_H

#include "habitrack/geometricType.h"
#include "habitrack/transformation.h"
#include <opencv2/core.hpp>

namespace ht
{
class ImageContainer;
class FeatureContainer;
class MatchesContainer;
}

/* namespace ceres */
/* { */
/* class Problem; */
/* } */

namespace ht
{
class PanoramaStitcher
{
public:
    PanoramaStitcher(std::shared_ptr<ImageContainer> imgContainer,
        std::shared_ptr<FeatureContainer> ftContainer,
        std::shared_ptr<MatchesContainer> matchContainer, const std::vector<size_t>& keyFrames,
        GeometricType type, bool blend);
    /* const cv::Mat& camMat, const cv::Mat& distCoeffs); */

    /* cv::Mat optimizeGlobal(); */
    /* std::tuple<cv::Mat, cv::Mat, cv::Mat> generatePano(cv::Size targetSize); */
private:
    /* std::vector<double> getCamParameterization(); */
    /* std::vector<double> getDistParameterization(); */
    /* std::vector<double> invertDistParameterization(const std::vector<double>& params); */

    cv::Rect2d generateBoundingRect(const std::vector<cv::Mat>& trafos, const cv::Size& imageSize);
    cv::Rect2d generateBoundingRectHelper(
        const cv::Mat& trafo, cv::Size size, cv::Rect2d currRect = cv::Rect2d());
    /* cv::Mat draw(const cv::Mat& trafo, size_t idI, size_t idJ); */

    /* void addFunctor(ceres::Problem& problem, const cv::Point2f& ptI, */
    /*     const cv::Point2f& ptJ, cv::Mat* trafoI, cv::Mat* trafoJ, double* camParams, */
    /*     double* distParams, std::vector<double>* paramsI, std::vector<double>* paramsJ); */
    /* void repairTrafos(std::vector<cv::Mat>& trafos, */
    /*    const std::vector<std::vector<double>>& params); */
    /* void repairIntriniscs(const std::vector<double>& camParams, */
    /*     const std::vector<double>& distParams); */
    /* std::tuple<cv::Mat, std::vector<cv::Point2f>, std::vector<cv::Point2f>> getTransformation( */
    /*     size_t idI, size_t idJ); */

    /* std::tuple<cv::Mat, cv::Mat, cv::Mat> stitchPano(cv::Rect2d rect, */
    /*     const std::vector<cv::Mat>& trafos, cv::Size targetSize); */

private:
    std::shared_ptr<ImageContainer> mImgContainer;
    std::shared_ptr<FeatureContainer> mFtContainer;
    std::shared_ptr<MatchesContainer> mMatchContainer;
    std::vector<size_t> mKeyFrames;
    GeometricType mType;
    bool mBlend;

    /* cv::Mat mCamMat; */
    /* cv::Mat mCamMatInv; */
    /* cv::Mat mDistCoeffs; */

    /* std::vector<cv::Mat> globalTrafos; */
};
} // namespace ht
#endif // HABITRACK_PANORAMA_STITCHER_H
