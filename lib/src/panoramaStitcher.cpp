#include "habitrack/panoramaStitcher.h"

#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <unsupported/Eigen/MatrixFunctions>

#include <ceres/ceres.h>

#include "affinityGlobalOptimizer.h"
#include "habitrack/isometry.h"
#include "homographyGlobalOptimizer.h"
#include "isometryGlobalOptimizer.h"
#include "progressBar.h"
#include "similarityGlobalOptimizer.h"

using Gt = ht::GeometricType;
namespace ht
{
PanoramaStitcher::PanoramaStitcher(std::shared_ptr<BaseImageContainer> imgContainer,
    std::shared_ptr<BaseFeatureContainer> ftContainer, const PairwiseMatches& matches,
    const PairwiseTrafos& trafos, const std::vector<size_t>& keyFrames, GeometricType type,
    Blending blend, const cv::Mat& camMat, const cv::Mat& distCoeffs)
    : mImgContainer(std::move(imgContainer))
    , mFtContainer(std::move(ftContainer))
    , mMatches(matches)
    , mTrafos(trafos)
    , mKeyFrames(keyFrames)
    , mKeyFramesSet(std::begin(keyFrames), std::end(keyFrames))
    , mType(type)
    , mBlend(blend == Blending::Blend ? true : false)
    , mCamMat(camMat)
    , mCamMatInv()
    , mDistCoeffs(distCoeffs)
    , mOptimizedTrafos(mImgContainer->getNumImgs(), cv::Mat())
{
    if (mCamMat.empty())
    {
        mCamMat = cv::Mat::eye(3, 3, CV_64F);
        /* auto imgSize = mImgContainer->getImgSize(); */
        /* mCamMat.at<double>(0, 0) = mCamMat.at<double>(1, 1) */
        /*     = 1.2 * std::max(imgSize.width, imgSize.height); */
        /* mCamMat.at<double>(0, 2) = imgSize.width / 2; */
        /* mCamMat.at<double>(1, 2) = imgSize.height / 2; */
        mDistCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    }
    // zero out tangential components
    mDistCoeffs.at<double>(2, 0) = mDistCoeffs.at<double>(3, 0) = 0.0;
    cv::invert(mCamMat, mCamMatInv);
}

PanoramaStitcher::PanoramaStitcher(std::shared_ptr<BaseImageContainer> imgContainer,
    std::shared_ptr<BaseFeatureContainer> ftContainer,
    std::shared_ptr<MatchesContainer> matchesContainer, const std::vector<size_t>& keyFrames,
    GeometricType type, Blending blend, const cv::Mat& camMat, const cv::Mat& distCoeffs)
    : PanoramaStitcher(imgContainer, ftContainer, matchesContainer->getMatches(type),
        matchesContainer->getTrafos(type), keyFrames, type, blend, camMat, distCoeffs)
{
}

// TODO: better init with num-fts-weighted loop graph
void PanoramaStitcher::initTrafos()
{
    for (std::size_t i = 0; i < mKeyFrames.size(); i++)
        mOptimizedTrafos[mKeyFrames[i]] = getIdentity();

    for (std::size_t i = 1; i < mKeyFrames.size(); i++)
    {
        auto prevId = mKeyFrames[i - 1];
        auto currId = mKeyFrames[i];

        // get trafo to previous image
        auto trafo = mTrafos.at(std::make_pair(prevId, currId));

        // chain transforamtion with previous to get trafo relative to first frame
        trafo = invertSpecial(trafo, mType);
        trafo = mOptimizedTrafos[prevId] * trafo;
        mOptimizedTrafos[currId] = trafo;
    }
}

cv::Mat PanoramaStitcher::transformBoundingRect(const cv::Mat& trafo) const
{
    auto size = mImgContainer->getImgSize();
    cv::Mat corners = (cv::Mat_<double>(4, 2) << 0.0, 0.0, size.width - 1, 0.0, 0.0,
        size.height - 1, size.width - 1, size.height - 1);

    cv::Mat cornersHomo;
    cv::convertPointsToHomogeneous(corners, cornersHomo);

    for (int i = 0; i < cornersHomo.rows; i++)
    {
        auto vec = cornersHomo.at<cv::Vec3d>(i, 0);
        cv::Mat vecMat = trafo * cv::Mat(vec);
        cornersHomo.at<cv::Vec3d>(i, 0) = vecMat;
    }

    cv::Mat cornersTrafo;
    cv::convertPointsFromHomogeneous(cornersHomo, cornersTrafo);

    return cornersTrafo;
}

cv::Point PanoramaStitcher::getCenter(const cv::Mat& trafo)
{
    auto cornersTrafo = transformBoundingRect(trafo);
    cv::Point2f p0(cornersTrafo.at<double>(0, 0), cornersTrafo.at<double>(0, 1));
    cv::Point2f p1(cornersTrafo.at<double>(3, 0), cornersTrafo.at<double>(3, 1));
    return 0.5 * (p0 + p1);
}

std::tuple<cv::Mat, cv::Mat, cv::Mat> PanoramaStitcher::stitchPano(
    cv::Size targetSize, bool drawCenters)
{
    auto rect = generateBoundingRect();
    rect.width = rect.width - rect.x;
    rect.height = rect.height - rect.y;

    double scale = 1.0;
    if (rect.width > targetSize.width)
        scale = std::min(targetSize.width / rect.width, scale);
    if (rect.height > targetSize.height)
        scale = std::min(targetSize.height / rect.height, scale);

    cv::Mat scaleMat = getScaleMat(scale);
    cv::Mat transMat = getTranslationMat(-rect.x, -rect.y);
    cv::Size newSize(scale * rect.width, scale * rect.height);

    std::cout << "Target size: " << newSize << std::endl;

    cv::detail::MultiBandBlender blender(false, 1);
    blender.prepare(cv::Rect(0, 0, newSize.width, newSize.height));

    // here we start from 0 instead of 1 because we explicitly added the identity trafo
    cv::Mat img0;
    std::vector<cv::Point> centers;
    centers.reserve(mKeyFrames.size());
    for (size_t i = 0; i < mKeyFrames.size(); i++)
    {
        auto currId = mKeyFrames[i];
        cv::Mat currImg = mImgContainer->at(mKeyFrames[i]);
        cv::Mat mask(currImg.size(), CV_8UC1, cv::Scalar(255));

        cv::Mat undistImg;
        cv::Mat undistMask;
        cv::undistort(currImg, undistImg, mCamMat, mDistCoeffs, mCamMat);
        cv::undistort(mask, undistMask, mCamMat, mDistCoeffs, mCamMat);

        auto currTrafo = scaleMat * transMat * mCamMat * mOptimizedTrafos[currId] * mCamMatInv;
        cv::Mat warpedMask, warped;
        cv::warpPerspective(undistMask, warpedMask, currTrafo, newSize);
        cv::warpPerspective(undistImg, warped, currTrafo, newSize);

        // DEBUG
        if (drawCenters)
        {
            if (i > 0)
            {
                // TODO: if-else
                for (std::size_t j = mKeyFrames[i - 1] + 1; j < mKeyFrames[i]; j++)
                {
                    auto interpTrafo
                        = scaleMat * transMat * mCamMat * mOptimizedTrafos[j] * mCamMatInv;
                    centers.push_back(getCenter(interpTrafo));
                }
            }
            centers.push_back(getCenter(currTrafo));
        }
        // DEBUG

        if (mBlend)
            blender.feed(warped, warpedMask, cv::Point(0, 0));
        else
        {
            if (i == 0)
                img0 = warped;
            warped.copyTo(img0, warpedMask);

            // for debugging
            /* cv::namedWindow("current pano", cv::WINDOW_NORMAL); */
            /* cv::imshow("bla", img0); */
            /* std::cout << mKeyFrames[i] << std::endl; */
            /* while (cv::waitKey(0) != 27) */
            /* { */
            /* } */
        }
    }

    cv::Mat pano;
    if (mBlend)
    {
        cv::Mat res, resMask;
        blender.blend(res, resMask);
        res.convertTo(pano, CV_8U);
    }
    else
        pano = img0;

    // DEBUG
    if (drawCenters)
    {
        for (std::size_t i = 0; i < centers.size(); i++)
        {
            auto center = centers[i];
            cv::drawMarker(pano, center, cv::Scalar(0, 255, 0));

            if (i > 0)
                cv::line(pano, centers[i - 1], centers[i], cv::Scalar(0, 255, 0));
        }
    }
    // DEBUG
    return std::make_tuple(pano, scaleMat, transMat);
}

void PanoramaStitcher::globalOptimize(FramesMode framesMode, KeyFramesMode keyFramesMode)
{
    auto camParams = getCamParameterization();
    auto distParams = getDistParameterization();
    // invert params to use the model as an undistortion model instead of an distortion model
    distParams = invertDistParameterization(distParams);

    std::vector<std::vector<double>> params;
    if (mType == GeometricType::Isometry)
    {
        params = std::vector<std::vector<double>>(
            mImgContainer->getNumImgs(), std::vector<double>(3, -11));
    }

    ceres::Problem problem;
    ceres::Solver::Options options;
    options.num_threads = 16;
    options.max_num_iterations = 500;
    options.minimizer_progress_to_stdout = true;

    std::cout << "Building Optimization Problem..." << std::endl;
    ProgressBar bar(mMatches.size());

    std::size_t numFunctors = 0;
    for (const auto& [pair, match] : mMatches)
    {
        auto [idI, idJ] = pair;
        cv::Mat* trafoI = &mOptimizedTrafos[idI];
        cv::Mat* trafoJ = &mOptimizedTrafos[idJ];

        if (framesMode == FramesMode::KeyFramesOnly && (!isKeyFrame(idI) || !isKeyFrame(idJ)))
        {
            ++bar;
            continue;
        }

        auto [ftsI, ftsJ] = getCorrespondingPoints(pair, match);
        for (size_t k = 0; k < ftsI.size(); k++)
        {
            addFunctor(problem, ftsI[k].pt, ftsJ[k].pt, trafoI, trafoJ, camParams.data(),
                distParams.data(), &params[idI], &params[idJ], ftsI[k].response * ftsJ[k].response);

            if (keyFramesMode == KeyFramesMode::Fixed && isKeyFrame(idI))
                problem.SetParameterBlockConstant(trafoI->ptr<double>(0));
            if (keyFramesMode == KeyFramesMode::Fixed && isKeyFrame(idJ))
                problem.SetParameterBlockConstant(trafoJ->ptr<double>(0));

            numFunctors++;
        }
        ++bar;
        bar.display();
    }
    bar.done();

    std::cout << "num functors: " << numFunctors << std::endl;

    std::cout << "Optimizing Problem..." << std::endl;

    problem.SetParameterBlockConstant(camParams.data());
    problem.SetParameterBlockConstant(distParams.data());

    problem.SetParameterBlockConstant(mOptimizedTrafos[mKeyFrames[0]].ptr<double>(0));
    /* problem.SetParameterBlockConstant(params[mKeyFrames[0]].data()); */

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    repairTrafos(params, framesMode);

    /* std::cout << mCamMat << std::endl; */
    /* std::cout << mDistCoeffs << std::endl; */

    /* return std::get<0>(stitchPano(boundingRect, globalTrafos, targetSize)); */
}

void PanoramaStitcher::reintegrate()
{
    for (std::size_t i = 1; i < mKeyFrames.size(); i++)
    {
        auto prevKf = mKeyFrames[i - 1];
        auto currKf = mKeyFrames[i];

        for (std::size_t j = prevKf + 1; j < currKf; j++)
        {
            double alpha = static_cast<double>(j - prevKf) / (currKf - prevKf);
            mOptimizedTrafos[j]
                = interpolateTrafo(alpha, mOptimizedTrafos[prevKf], mOptimizedTrafos[currKf]);
        }
    }
}

std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>>
PanoramaStitcher::getCorrespondingPoints(
    std::pair<std::size_t, std::size_t> pair, const Matches& matches)
{
    std::vector<cv::KeyPoint> src, dst;
    src.reserve(matches.size());
    dst.reserve(matches.size());
    for (const auto& match : matches)
    {
        src.push_back(mFtContainer->featureAt(pair.first)[match.queryIdx]);
        dst.push_back(mFtContainer->featureAt(pair.second)[match.trainIdx]);
    }

    return std::make_pair(src, dst);
}

cv::Mat PanoramaStitcher::interpolateTrafo(
    double alpha, const cv::Mat& mat1, const cv::Mat& mat2) const
{
    Eigen::Matrix3d eigenMat1, eigenMat2;
    cv::cv2eigen(mat1, eigenMat1);
    cv::cv2eigen(mat2, eigenMat2);

    Eigen::Matrix3d res = eigenMat1 * (eigenMat1.inverse() * eigenMat2).pow(alpha);

    cv::Mat interp;
    cv::eigen2cv(res, interp);
    return interp;
}

/* std::vector<size_t> getMatchedImgs(size_t currImg, const std::vector<std::pair<size_t, size_t>>&
 * matchPairs) */
/* { */
/*     std::vector<size_t> matchedIds; */
/*     for (const auto& pair : matchPairs) */
/*     { */
/*         if (pair.second == currImg) */
/*             matchedIds.push_back(pair.first); */
/*         if (pair.first >= currImg) */
/*             break; */
/*     } */
/*     return matchedIds; */
/* } */

void PanoramaStitcher::addFunctor(ceres::Problem& problem, const cv::Point2f& ptI,
    const cv::Point2f& ptJ, cv::Mat* trafoI, cv::Mat* trafoJ, double* camParams, double* distParams,
    std::vector<double>* paramsI, std::vector<double>* paramsJ, double weight)
{
    switch (mType)
    {
    case GeometricType::Isometry:
    {
        // Isometries need a parametrization vector because the angle needs to
        // be optimized explicitly !!!
        if ((*paramsI)[0] < -10)
        {
            double angle0 = std::atan2((*trafoI).at<double>(1, 0), (*trafoI).at<double>(0, 0));
            (*paramsI)[0] = angle0;
            (*paramsI)[1] = (*trafoI).at<double>(0, 2);
            (*paramsI)[2] = (*trafoI).at<double>(1, 2);
        }
        if ((*paramsJ)[0] < -10)
        {
            double angle1 = std::atan2((*trafoJ).at<double>(1, 0), (*trafoJ).at<double>(0, 0));
            (*paramsJ)[0] = angle1;
            (*paramsJ)[1] = (*trafoJ).at<double>(0, 2);
            (*paramsJ)[2] = (*trafoJ).at<double>(1, 2);
        }

        using Functor = IsometryGlobalFunctor;
        Functor* functor
            = new Functor(Eigen::Vector2d(ptI.x, ptI.y), Eigen::Vector2d(ptJ.x, ptJ.y), weight);

        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Functor, 4, 4, 3, 3, 3>(functor),
            new ceres::HuberLoss(1.0), camParams, distParams, static_cast<double*>(paramsI->data()),
            static_cast<double*>(paramsJ->data()));

        // no need to constraint them because values below zero or over 2*pi are still angles
        /* problem.SetParameterLowerBound(static_cast<double*>(paramsI->data()), 0, 0); */
        /* problem.SetParameterUpperBound(static_cast<double*>(paramsI->data()), 0, 2 * 3.14159); */
        /* problem.SetParameterLowerBound(static_cast<double*>(paramsJ->data()), 0, 0); */
        /* problem.SetParameterUpperBound(static_cast<double*>(paramsJ->data()), 0, 2 * 3.14159); */
        break;
    }
    case GeometricType::Similarity:
    {
        using Functor = SimilarityGlobalFunctor;
        Functor* functor
            = new Functor(Eigen::Vector2d(ptI.x, ptI.y), Eigen::Vector2d(ptJ.x, ptJ.y), weight);

        // use all first 6 elements although only 4 in total are needed
        // to avoid copying data and use "repairTrafos" to update ignored values
        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Functor, 4, 4, 3, 6, 6>(functor),
            new ceres::HuberLoss(1.0), camParams, distParams, trafoI->ptr<double>(0),
            trafoJ->ptr<double>(0));
        break;
    }
    case GeometricType::Affinity:
    {
        using Functor = AffinityGlobalFunctor;
        Functor* functor
            = new Functor(Eigen::Vector2d(ptI.x, ptI.y), Eigen::Vector2d(ptJ.x, ptJ.y), weight);

        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Functor, 4, 4, 3, 6, 6>(functor),
            new ceres::HuberLoss(1.0), camParams, distParams, trafoI->ptr<double>(0),
            trafoJ->ptr<double>(0));
        break;
    }
    case GeometricType::Homography:
    {
        using Functor = HomographyGlobalFunctor;
        Functor* functor
            = new Functor(Eigen::Vector2d(ptI.x, ptI.y), Eigen::Vector2d(ptJ.x, ptJ.y), weight);

        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Functor, 4, 4, 3, 9, 9>(functor),
            new ceres::HuberLoss(1.0), camParams, distParams, trafoI->ptr<double>(0),
            trafoJ->ptr<double>(0));
        break;
    }
    default:
        break;
    }
}

void PanoramaStitcher::repairTrafos(
    const std::vector<std::vector<double>>& params, FramesMode framesMode)
{
    switch (mType)
    {
    case GeometricType::Isometry:
        // rebuild matrix from params vector
        for (std::size_t i = 0; i < mImgContainer->getNumImgs(); i++)
        {
            if (framesMode == FramesMode::AllFrames || isKeyFrame(i))
            {
                mOptimizedTrafos[i].at<double>(0, 0) = std::cos(params[i][0]);
                mOptimizedTrafos[i].at<double>(0, 1) = std::sin(params[i][0]);
                mOptimizedTrafos[i].at<double>(1, 0) = -std::sin(params[i][0]);
                mOptimizedTrafos[i].at<double>(1, 1) = std::cos(params[i][0]);

                mOptimizedTrafos[i].at<double>(0, 2) = params[i][1];
                mOptimizedTrafos[i].at<double>(1, 2) = params[i][2];
            }
        }
        break;
    case GeometricType::Similarity:
        // ensure similartiy property
        for (std::size_t i = 0; i < mImgContainer->getNumImgs(); i++)
        {
            if (framesMode == FramesMode::AllFrames || isKeyFrame(i))
            {
                auto& trafo = mOptimizedTrafos[i];
                trafo.at<double>(1, 0) = -trafo.at<double>(0, 1);
                trafo.at<double>(1, 1) = trafo.at<double>(0, 0);
            }
        }
        break;
    case GeometricType::Affinity:
        break;
    case GeometricType::Homography:
        break;
    default:
        break;
    }
}

cv::Rect2d PanoramaStitcher::generateBoundingRect() const
{
    auto imgSize = mImgContainer->getImgSize();
    cv::Rect2d boundingRect(0, 0, imgSize.width, imgSize.height);

    // store trafos relative to first frame
    for (size_t i = 1; i < mKeyFrames.size(); i++)
    {
        auto currId = mKeyFrames[i];
        boundingRect = generateBoundingRectHelper(mOptimizedTrafos[currId], boundingRect);
    }
    return boundingRect;
}

cv::Rect2d PanoramaStitcher::generateBoundingRectHelper(
    const cv::Mat& trafo, cv::Rect2d currRect) const
{
    cv::Mat currTrafo = mCamMat * trafo * mCamMatInv;
    auto cornersTrafo = transformBoundingRect(currTrafo);

    std::vector<cv::Mat> cornersTrafoSplit(2);
    cv::split(cornersTrafo, cornersTrafoSplit);

    double minX, maxX, minY, maxY;
    cv::minMaxLoc(cornersTrafoSplit[0], &minX, &maxX);
    cv::minMaxLoc(cornersTrafoSplit[1], &minY, &maxY);

    currRect.x = std::min(currRect.x, minX);
    currRect.y = std::min(currRect.y, minY);
    currRect.width = std::max(currRect.width, maxX);
    currRect.height = std::max(currRect.height, maxY);

    return currRect;
}

std::vector<double> PanoramaStitcher::getCamParameterization() const
{
    std::vector<double> params;
    params.reserve(4);

    // fx, fy
    params.push_back(mCamMat.at<double>(0, 0));
    params.push_back(mCamMat.at<double>(1, 1));

    // px,  py
    params.push_back(mCamMat.at<double>(0, 2));
    params.push_back(mCamMat.at<double>(1, 2));
    return params;
}

std::vector<double> PanoramaStitcher::getDistParameterization() const
{
    std::vector<double> params;
    params.reserve(3);

    // skip tangential coeffs
    params.push_back(mDistCoeffs.at<double>(0, 0));
    params.push_back(mDistCoeffs.at<double>(1, 0));
    params.push_back(mDistCoeffs.at<double>(4, 0));
    return params;
}

std::vector<double> PanoramaStitcher::invertDistParameterization(
    const std::vector<double>& params) const
{
    std::vector<double> invParams;
    if (params.empty())
        return invParams;

    invParams.reserve(3);
    invParams.push_back(-params[0]);
    invParams.push_back(3 * params[0] * params[0] - params[1]);
    invParams.push_back(
        8 * params[0] * params[1] - 12 * params[0] * params[0] * params[0] - params[2]);
    return invParams;
}

void PanoramaStitcher::repairIntriniscs(
    const std::vector<double>& camParams, const std::vector<double>& distParams)
{
    mCamMat.at<double>(0, 0) = camParams[0];
    mCamMat.at<double>(1, 1) = camParams[1];
    mCamMat.at<double>(0, 2) = camParams[2];
    mCamMat.at<double>(1, 2) = camParams[3];
    cv::invert(mCamMat, mCamMatInv);

    auto invertedDistParams = invertDistParameterization(distParams);
    mDistCoeffs.at<double>(0, 0) = invertedDistParams[0];
    mDistCoeffs.at<double>(1, 0) = invertedDistParams[1];
    mDistCoeffs.at<double>(4, 0) = invertedDistParams[4];
}

cv::Mat PanoramaStitcher::draw(const cv::Mat& trafo, size_t idI, size_t idJ)
{
    auto pair = std::make_pair(idI, idJ);
    auto imgJ = mImgContainer->at(idJ);
    auto matches = mMatches[pair];
    auto [ftsI, ftsJ] = getCorrespondingPoints(pair, matches);

    std::vector<cv::Point2f> ptsI, ptsITrans;
    ptsI.reserve(ftsI.size());
    for (const auto& kp : ftsI)
        ptsI.push_back(kp.pt);

    cv::perspectiveTransform(ptsI, ptsITrans, trafo);

    for (size_t i = 0; i < ptsI.size(); i++)
    {
        cv::drawMarker(imgJ, ftsJ[i].pt, cv::Scalar(0, 255, 0));
        cv::drawMarker(imgJ, ptsITrans[i], cv::Scalar(0, 0, 255));
        cv::line(imgJ, ftsJ[i].pt, ptsITrans[i], cv::Scalar(0, 255, 255));
    }

    cv::putText(imgJ, std::to_string(idI) + " to " + std::to_string(idJ), cv::Point(200, 200),
        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0));

    cv::imshow("bla", imgJ);
    while (cv::waitKey(0) != 27)
    {
    }

    return imgJ;
}
} // namespace ht
