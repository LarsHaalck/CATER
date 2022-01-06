#include "panorama/panoramaStitcher.h"

#include "affinityGlobalOptimizer.h"
#include "homographyGlobalOptimizer.h"
#include "image-processing/idTranslator.h"
#include "image-processing/isometry.h"
#include "io/io.h"
#include "io/matIO.h"
#include "isometryGlobalOptimizer.h"
#include "progressbar/progressBar.h"
#include "similarityGlobalOptimizer.h"
#include "similarityGPSOptimizer.h"
#include "util/algorithm.h"
#include "util/stopWatch.h"

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <filesystem>
#include <fstream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>
#include <queue>
#include <spdlog/spdlog.h>
#include <unsupported/Eigen/MatrixFunctions>

#include <chrono>

namespace fs = std::filesystem;
using Gt = ht::GeometricType;
using namespace ht::matches;
using namespace ht::transformation;
using namespace ht::io;

namespace ht
{
PanoramaStitcher::PanoramaStitcher(const BaseImageContainer& images,
    const std::vector<size_t>& keyFrames, GeometricType type, const cv::Mat& camMat,
    const cv::Mat& distCoeffs)
    : mImages(images)
    , mKeyFrames(keyFrames)
    , mKeyFramesSet(std::begin(keyFrames), std::end(keyFrames))
    , mType(type)
    , mCamMat(camMat)
    , mCamMatInv()
    , mDistCoeffs(distCoeffs)
    , mOptimizedTrafos(mImages.size(), cv::Mat())
    , mOptimizedParams(mImages.size(), std::vector<double>(3, -11))
    , mReintegrated(false)
    , mGPSSim(cv::Mat::eye(3, 3, CV_64F))
{
    if (mCamMat.empty())
    {
        mCamMat = cv::Mat::eye(3, 3, CV_64F);

        auto imgSize = mImages.getImgSize();
        /* mCamMat.at<double>(0, 0) = mCamMat.at<double>(1, 1) */
        /*     = 1.2 * std::max(imgSize.width, imgSize.height); */
        mCamMat.at<double>(0, 2) = static_cast<double>(imgSize.width) / 2;
        mCamMat.at<double>(1, 2) = static_cast<double>(imgSize.height) / 2;

        mDistCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    }
    // zero out tangential components
    mDistCoeffs.at<double>(2, 0) = mDistCoeffs.at<double>(3, 0) = 0.0;
    cv::invert(mCamMat, mCamMatInv);
}

// TODO: better init with num-fts-weighted loop graph
void PanoramaStitcher::initTrafos(const PairwiseTrafos& trafos)
{
    for (std::size_t i = 0; i < mKeyFrames.size(); i++)
        mOptimizedTrafos[mKeyFrames[i]] = getIdentity(true);

    for (std::size_t i = 1; i < mKeyFrames.size(); i++)
    {
        auto prevId = mKeyFrames[i - 1];
        auto currId = mKeyFrames[i];

        // get trafo to previous image
        auto trafo = trafos.at(std::make_pair(prevId, currId));

        // chain transforamtion with previous to get trafo relative to first frame
        trafo = invert(trafo, mType, true);
        trafo = mOptimizedTrafos[prevId] * trafo;
        mOptimizedTrafos[currId] = trafo;
    }

    if (mType == GeometricType::Isometry)
        buildParamsVector();
}

void PanoramaStitcher::initTrafosMultipleHelper(std::size_t currBlock, const cv::Mat& currTrafo,
    const std::vector<cv::Mat>& localOptimalTrafos, const std::vector<std::size_t>& sizes)
{
    Translator translator(sizes);
    auto lower = translator.localToGlobal(std::make_pair(currBlock, static_cast<std::size_t>(0)));
    auto upper = lower + sizes[currBlock];
    for (std::size_t i = lower; i < upper; i++)
    {
        if (isKeyFrame(i))
            mOptimizedTrafos[i] = currTrafo * localOptimalTrafos[i - lower];
    }
}

void PanoramaStitcher::initTrafosFromMultipleVideos(const PairwiseTrafos& trafos,
    const std::vector<std::size_t> sizes,
    const std::vector<std::vector<cv::Mat>>& localOptimalTrafos,
    const std::unordered_map<std::pair<std::size_t, std::size_t>,
        std::pair<std::size_t, std::size_t>, hash>& optimalTransitions)
{
    for (std::size_t i = 0; i < mKeyFrames.size(); i++)
        mOptimizedTrafos[mKeyFrames[i]] = getIdentity(true);

    std::queue<std::pair<std::size_t, cv::Mat>> queue;
    queue.push(std::make_pair(0, getIdentity(true)));

    std::unordered_set<std::size_t> marked;
    Translator translator(sizes);
    while (!queue.empty())
    {
        // find all descendants of current node
        auto [currBlock, currTrafo] = queue.front();
        initTrafosMultipleHelper(currBlock, currTrafo, localOptimalTrafos[currBlock], sizes);
        queue.pop();
        marked.insert(currBlock);
        for (const auto& [transBlock, transFrame] : optimalTransitions)
        {
            if (transBlock.first == currBlock && !marked.count(transBlock.second))
            {
                cv::Mat pairTrans = invert(trafos.at(transFrame), mType, true);

                auto localId = translator.globalToLocal(transFrame.second).second;
                cv::Mat preMult = localOptimalTrafos[transBlock.second][localId];
                cv::Mat postMult = mOptimizedTrafos[transFrame.first];

                cv::Mat invertPreMult;
                cv::invert(preMult, invertPreMult);
                cv::Mat newTrans = postMult * pairTrans * invertPreMult;

                queue.push(std::make_pair(transBlock.second, newTrans));
            }
            else if (transBlock.second == currBlock && !marked.count(transBlock.first))
            {
                cv::Mat pairTrans = makeFull(trafos.at(transFrame));

                auto localId = translator.globalToLocal(transFrame.first).second;
                cv::Mat preMult = localOptimalTrafos[transBlock.first][localId];
                cv::Mat postMult = mOptimizedTrafos[transFrame.second];

                cv::Mat invertPreMult;
                cv::invert(preMult, invertPreMult);
                cv::Mat newTrans = postMult * pairTrans * invertPreMult;

                queue.push(std::make_pair(transBlock.first, newTrans));
            }
        }
    }
    if (mType == GeometricType::Isometry)
        buildParamsVector();
}

cv::Mat PanoramaStitcher::transformBoundingRect(const cv::Mat& trafo) const
{
    auto size = mImages.getImgSize();
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

void PanoramaStitcher::buildParamsVector()
{
    for (std::size_t i = 0; i < mImages.size(); i++)
    {
        const auto& trafoI = mOptimizedTrafos[i];
        auto& paramsI = mOptimizedParams[i];
        if (!mOptimizedTrafos[i].empty())
        {
            double angle0 = std::atan2(trafoI.at<double>(1, 0), trafoI.at<double>(0, 0));
            paramsI[0] = angle0;
            paramsI[1] = trafoI.at<double>(0, 2);
            paramsI[2] = trafoI.at<double>(1, 2);
        }
    }
}

std::tuple<cv::Mat, cv::Mat, cv::Size> PanoramaStitcher::scaleTransMat(cv::Size targetSize) const
{
    auto rect = generateBoundingRect();
    rect.width = rect.width - rect.x;
    rect.height = rect.height - rect.y;

    double scale = 1.0;
    if (rect.width > targetSize.width)
        scale = std::min(targetSize.width / rect.width, scale);
    if (rect.height > targetSize.height)
        scale = std::min(targetSize.height / rect.height, scale);

    cv::Mat scaleMat = getScaleMat(scale, true);
    cv::Mat transMat = getTranslationMat(-rect.x, -rect.y, true);
    cv::Size newSize(scale * rect.width, scale * rect.height);

    return std::make_tuple(scaleMat, transMat, newSize);
}

std::tuple<cv::Mat, std::vector<cv::Mat>> PanoramaStitcher::stitchPano(
    cv::Size targetSize, bool blend, std::shared_ptr<BaseProgressBar> cb) const
{
    auto [scaleMat, transMat, newSize] = scaleTransMat(targetSize);
    spdlog::info("Target size: {} x {}", newSize.width, newSize.height);

    cv::detail::MultiBandBlender blender(false, 5);
    blender.prepare(cv::Rect(0, 0, newSize.width, newSize.height));

    spdlog::info("Stitching Panorama");

    if (!cb)
        cb = std::make_shared<ProgressBar>();
    cb->status("Stitching Panorama");
    cb->setTotal(mKeyFrames.size());

    // here we start from 0 instead of 1 because we explicitly added the identity trafo
    cv::Mat img0;
    std::vector<cv::Mat> trafos;
    PreciseStopWatch timer;
    for (std::size_t i = 0; i < mKeyFrames.size(); i++)
    {
        auto currId = mKeyFrames[i];
        cv::Mat currImg = mImages.at(mKeyFrames[i]);

        cv::Mat mask(currImg.size(), CV_8UC1, cv::Scalar(255));

        cv::Mat undistImg;
        cv::Mat undistMask;
        cv::undistort(currImg, undistImg, mCamMat, mDistCoeffs, mCamMat);
        cv::undistort(mask, undistMask, mCamMat, mDistCoeffs, mCamMat);

        auto preTrafo = scaleMat * transMat * mCamMat;
        auto currTrafo = preTrafo * mOptimizedTrafos[currId] * mCamMatInv;
        cv::Mat warpedMask, warped;
        cv::warpPerspective(undistMask, warpedMask, currTrafo, newSize);
        cv::warpPerspective(undistImg, warped, currTrafo, newSize);

        if (mReintegrated && i > 0)
        {
            for (std::size_t j = mKeyFrames[i - 1] + 1; j < mKeyFrames[i]; j++)
            {
                auto interpTrafo = preTrafo * mOptimizedTrafos[j] * mCamMatInv;
                trafos.push_back(interpTrafo);
            }
        }
        trafos.push_back(currTrafo);

        if (blend)
            blender.feed(warped, warpedMask, cv::Point(0, 0));
        else
        {
            if (i == 0)
                img0 = warped;
            warped.copyTo(img0, warpedMask);
        }
        cb->inc();
    }

    cv::Mat pano;
    if (blend)
    {
        cv::Mat res, resMask;
        blender.blend(res, resMask);
        res.convertTo(pano, CV_8U);
    }
    else
        pano = img0;

    return std::make_tuple(pano, trafos);
}

void PanoramaStitcher::globalOptimizeKeyFrames(const BaseFeatureContainer& fts,
    const PairwiseMatches& matches, std::size_t limitTo, const GPSMap& gps,
    std::shared_ptr<BaseProgressBar> cb)
{
    if (!cb)
        cb = std::make_shared<ProgressBar>();

    PreciseStopWatch timer;
    bool res = globalOptimize(fts, matches, FramesMode::KeyFramesOnly, limitTo, true, gps, cb);
    reconstructTrafos(FramesMode::KeyFramesOnly);
    auto elapsed_time = timer.elapsed_time<unsigned int, std::chrono::milliseconds>();
    spdlog::info("Optimization took {} ms", elapsed_time);

    if (!res)
        spdlog::warn("Optimization might not be reliable");
}

void PanoramaStitcher::refineNonKeyFrames(const BaseFeatureContainer& fts,
    const PairwiseMatches& matches, std::size_t limitTo, std::shared_ptr<BaseProgressBar> cb)
{
    auto pairs = getKeyList(matches);

    std::vector<std::size_t> postpones;

    std::vector<PairwiseMatches> boundedMatches;
    std::size_t currMatchId = 0;
    for (std::size_t i = 1; i < mKeyFrames.size(); i++)
    {
        auto prevKf = mKeyFrames[i - 1];
        auto currKf = mKeyFrames[i];
        auto pair = pairs[currMatchId];
        PairwiseMatches currMatches;
        while (pair.first >= prevKf && pair.second <= currKf && currMatchId < pairs.size())
        {
            currMatches.insert(std::make_pair(pair, matches.at(pair)));
            currMatchId++;
            pair = pairs[currMatchId];
        }

        // save all frames, without matches with the adjacent keyframes for later refinement
        for (std::size_t j = prevKf + 1; j < currKf; j++)
        {
            if (currMatches.count({prevKf, j}) + currMatches.count({j, currKf}) == 0)
            {
                spdlog::debug("Inserting: {} for postponed initialization", j);
                postpones.push_back(j);
            }
        }

        // should not be empty except when prevKf + 1 = currKf
        if (!currMatches.empty())
            boundedMatches.push_back(std::move(currMatches));
    }

    if (!cb)
        cb = std::make_shared<ProgressBar>();
    spdlog::info("Performing global optimization between keyframes");
    cb->status("Performing global optimization between keyframes");
    cb->setTotal(mKeyFrames.size() - 1);

    std::vector<bool> res(boundedMatches.size());
    PreciseStopWatch timer;
// optimization problems might have very different sizes so use dynamic scheduling
#pragma omp parallel for schedule(dynamic)
    for (std::size_t i = 0; i < boundedMatches.size(); i++)
    {
        res[i] = globalOptimize(fts, boundedMatches[i], FramesMode::AllFrames, limitTo, false);

#pragma omp critical
        cb->inc();
    }

    auto elapsed_time = timer.elapsed_time<unsigned int, std::chrono::milliseconds>();
    cb->status("Finished");
    cb->done();
    spdlog::info("Computed Features in {} ms", elapsed_time);
    reconstructTrafos(FramesMode::AllFrames);

    for (std::size_t i = 0; i < boundedMatches.size(); i++)
    {
        if (!res[i])
        {
            auto id = getKeyList(boundedMatches[i])[0].first;
            spdlog::warn(
                "Optimization might not be reliable for segmet starting with frame {}", id);
        }
    }

    spdlog::info("Interpolating postponed transformations of {} frames", postpones.size());
    if (postpones.empty())
        return;

    std::vector<int> left(postpones.size(), 0);
    std::vector<int> right(postpones.size(), 0);

    left[0] = postpones[0] - 1; // always correct
    for (std::size_t i = 1; i < postpones.size(); i++)
    {
        if (postpones[i - 1] + 1 == postpones[i])
            left[i] = left[i - 1];
        else
            left[i] = postpones[i] - 1;
    }

    // may need correction after the following loop
    right[postpones.size() - 1] = postpones[postpones.size() - 1] + 1;
    for (int i = static_cast<int>(postpones.size()) - 1; i >= 0; i--)
    {
        if (postpones[i + 1] - 1 == postpones[i])
            right[i] = right[i + 1];
        else
            right[i] = postpones[i] + 1;
    }

    for (std::size_t i = 0; i < postpones.size(); i++)
    {
        if (right[i] >= static_cast<int>(mImages.size()))
        {
            right[i] = left[i];
            left[i] = left[i] - 1;
        }
        if (left[i] < 0 || left[i] >= static_cast<int>(mImages.size()) || right[i] < 0
            || right[i] >= static_cast<int>(mImages.size()))
        {
            spdlog::warn("Boundaries have wrong values in postponed initialization");
        }

        auto alpha = static_cast<float>((postpones[i] - left[i])) / (right[i] - left[i]);
        spdlog::debug(
            "Postpone refine {}: {} -> {}, s = {}", postpones[i], left[i], right[i], alpha);
        mOptimizedTrafos[postpones[i]]
            = interpolateTrafo(alpha, mOptimizedTrafos[left[i]], mOptimizedTrafos[right[i]]);
    }
}

std::vector<std::size_t> PanoramaStitcher::sortIdsByResponseProduct(
    const std::vector<cv::KeyPoint>& ftsI, const std::vector<cv::KeyPoint>& ftsJ,
    const std::vector<float>& weights)
{
    auto ids = std::vector<std::size_t>(ftsI.size());
    std::iota(std::begin(ids), std::end(ids), 0);

    if (weights.empty())
    {
        std::sort(std::begin(ids), std::end(ids),
            [&](std::size_t k, std::size_t l) {
                return (ftsI[k].response * ftsJ[k].response)
                    > (ftsI[l].response * ftsJ[l].response);
            });
    }
    else
    {
        std::sort(std::begin(ids), std::end(ids),
            [&](std::size_t k, std::size_t l) { return weights[k] > weights[l]; });
    }

    return ids;
}

bool PanoramaStitcher::globalOptimize(const BaseFeatureContainer& fts,
    const PairwiseMatches& matches, FramesMode framesMode, std::size_t limitTo, bool multiThread,
    const GPSMap& gps, std::shared_ptr<BaseProgressBar> cb)
{
    auto camParams = getCamParameterization();
    auto distParams = getDistParameterization();
    // invert params to use the model as an undistortion model instead of an distortion model
    distParams = invertDistParameterization(distParams);

    ceres::Problem problem;
    ceres::Solver::Options options;
    options.max_num_iterations = 500;
    options.minimizer_progress_to_stdout = false;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.preconditioner_type = ceres::SCHUR_JACOBI;

    if (multiThread)
        options.num_threads = std::thread::hardware_concurrency();
    else
        options.num_threads = 1;

    if (cb)
    {
        spdlog::info("Building optimization problem");
        cb->status("Building optimization problem");
        cb->setTotal(matches.size());
    }

    for (const auto& [pair, match] : matches)
    {
        auto [idI, idJ] = pair;
        auto* trafoI = &mOptimizedTrafos[idI];
        auto* trafoJ = &mOptimizedTrafos[idJ];
        auto* paramsI = &mOptimizedParams[idI];
        auto* paramsJ = &mOptimizedParams[idJ];

        auto [ftsI, ftsJ, weights] = getCorrespondingPoints(pair, match, fts);

        auto numFunctors = ftsI.size();
        if (limitTo)
        {
            // get first limitTo features with hightes response (best features)
            auto p = sortIdsByResponseProduct(ftsI, ftsJ, weights);
            ftsI = permute(ftsI, p);
            ftsJ = permute(ftsJ, p);
            weights = permute(weights, p);
            numFunctors = std::min(limitTo, numFunctors);
        }

        for (size_t k = 0; k < numFunctors; k++)
        {
            float w = 0;
            if (!weights.empty())
                w = weights[k];
            addFunctor(problem, ftsI[k].pt, ftsJ[k].pt, trafoI, trafoJ, camParams.data(),
                distParams.data(), paramsI, paramsJ, ftsI[k].response * ftsJ[k].response, w);
        }
        if (framesMode == FramesMode::AllFrames && isKeyFrame(idI))
        {
            if (mType == GeometricType::Isometry)
                problem.SetParameterBlockConstant(paramsI->data());
            else
                problem.SetParameterBlockConstant(trafoI->ptr<double>(0));
        }
        if (framesMode == FramesMode::AllFrames && isKeyFrame(idJ))
        {
            if (mType == GeometricType::Isometry)
                problem.SetParameterBlockConstant(paramsJ->data());
            else
                problem.SetParameterBlockConstant(trafoJ->ptr<double>(0));
        }

        if (cb)
            cb->inc();
    }

    if (cb)
    {
        cb->status("Finished");
        cb->done();
    }

    if (framesMode == FramesMode::KeyFramesOnly)
        addGPSRegularizer(problem, gps);

    // fix first transformation to identity
    if (framesMode == FramesMode::KeyFramesOnly)
    {
        if (mType == GeometricType::Isometry)
            problem.SetParameterBlockConstant(mOptimizedParams[mKeyFrames[0]].data());
        else
            problem.SetParameterBlockConstant(mOptimizedTrafos[mKeyFrames[0]].ptr<double>(0));
    }

    // TODO: control this via argument
    // and should always be constant for AllFrames Mode for parallel processing
    problem.SetParameterBlockConstant(camParams.data());
    problem.SetParameterBlockConstant(distParams.data());

    /* std::cout << "Optimizing Problem..." << std::endl; */
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    /* std::cout << summary.FullReport() << std::endl; */

    /* repairIntriniscs(camParams, distParams); */
    return summary.IsSolutionUsable();
}

void PanoramaStitcher::addGPSRegularizer(ceres::Problem& problem, const GPSMap& gps)
{
    for (auto kf : mKeyFrames)
    {
        if (gps.count(kf) == 0)
            continue;

        using Functor = SimilarityGPSFunctor;
        auto pt = gps.at(kf);
        Functor* functor = new Functor(Eigen::Vector2d(pt.x, pt.y));

        auto& params = mOptimizedParams[kf];
        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Functor, 4, 6, 6>(functor),
            new ceres::HuberLoss(1.0), static_cast<double*>(params.data()),
            mGPSSim.ptr<double>(0));
    }
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
    mReintegrated = true;
}

std::tuple<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>, std::vector<float>>
PanoramaStitcher::getCorrespondingPoints(std::pair<std::size_t, std::size_t> pair,
    const Matches& matches, const BaseFeatureContainer& fts)
{
    std::vector<cv::KeyPoint> src, dst;
    src.reserve(matches.size());
    dst.reserve(matches.size());

    std::vector<float> weight;
    if (fts.getFeatureType() == FeatureType::SuperPoint)
        weight.reserve(matches.size());
    for (const auto& match : matches)
    {
        src.push_back(fts.featureAt(pair.first)[match.queryIdx]);
        dst.push_back(fts.featureAt(pair.second)[match.trainIdx]);

        if (fts.getFeatureType() == FeatureType::SuperPoint)
            weight.push_back(match.distance);
    }

    return std::make_tuple(src, dst, weight);
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

void PanoramaStitcher::addFunctor(ceres::Problem& problem, const cv::Point2f& ptI,
    const cv::Point2f& ptJ, cv::Mat* trafoI, cv::Mat* trafoJ, double* camParams, double* distParams,
    std::vector<double>* paramsI, std::vector<double>* paramsJ, float response, float weight)
{
    if (weight == 0.0f)
        weight = response;

    switch (mType)
    {
    case GeometricType::Isometry:
    {
        // Isometries need a parametrization vector because the angle needs to
        // be optimized explicitly
        // -11 means not initialized yet
        /* if ((*paramsI)[0] < -10) */
        /* { */
        /*     double angle0 = std::atan2((*trafoI).at<double>(1, 0), (*trafoI).at<double>(0, 0)); */
        /*     (*paramsI)[0] = angle0; */
        /*     (*paramsI)[1] = (*trafoI).at<double>(0, 2); */
        /*     (*paramsI)[2] = (*trafoI).at<double>(1, 2); */
        /* } */
        /* if ((*paramsJ)[0] < -10) */
        /* { */
        /*     double angle1 = std::atan2((*trafoJ).at<double>(1, 0), (*trafoJ).at<double>(0, 0)); */
        /*     (*paramsJ)[0] = angle1; */
        /*     (*paramsJ)[1] = (*trafoJ).at<double>(0, 2); */
        /*     (*paramsJ)[2] = (*trafoJ).at<double>(1, 2); */
        /* } */

        using Functor = IsometryGlobalFunctor;
        Functor* functor
            = new Functor(Eigen::Vector2d(ptI.x, ptI.y), Eigen::Vector2d(ptJ.x, ptJ.y), weight);

        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Functor, 4, 4, 3, 3, 3>(functor),
            new ceres::HuberLoss(1.0), camParams, distParams, static_cast<double*>(paramsI->data()),
            static_cast<double*>(paramsJ->data()));

        break;
    }
    case GeometricType::Similarity:
    {
        using Functor = SimilarityGlobalFunctor;
        Functor* functor
            = new Functor(Eigen::Vector2d(ptI.x, ptI.y), Eigen::Vector2d(ptJ.x, ptJ.y), weight);

        // use all first 6 elements although only 4 in total are needed
        // to avoid copying data and use "reconstructTrafos" to update ignored values
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

void PanoramaStitcher::reconstructTrafos(FramesMode framesMode)
{
    switch (mType)
    {
    case GeometricType::Isometry:
    {
        // rebuild matrix from params vector
        const auto& params = mOptimizedParams;
        for (std::size_t i = 0; i < mImages.size(); i++)
        {
            if ((framesMode == FramesMode::AllFrames && !isKeyFrame(i))
                || (framesMode == FramesMode::KeyFramesOnly && isKeyFrame(i)))
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
    }
    case GeometricType::Similarity:
        // ensure similarity property
        for (std::size_t i = 0; i < mImages.size(); i++)
        {
            if ((framesMode == FramesMode::AllFrames && !isKeyFrame(i))
                || (framesMode == FramesMode::KeyFramesOnly && isKeyFrame(i)))
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
    auto imgSize = mImages.getImgSize();
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

void PanoramaStitcher::loadTrafos(const fs::path& file)
{
    mOptimizedTrafos = getTrafos(file);
    if (mType == GeometricType::Isometry)
        buildParamsVector();
}

std::vector<cv::Mat> PanoramaStitcher::getTrafos(const fs::path& file)
{
    std::vector<cv::Mat> trafos;
    std::ifstream stream(file.string(), std::ios::in | std::ios::binary);
    checkStream(stream, file);
    {
        cereal::PortableBinaryInputArchive archive(stream);
        archive(trafos);
    }
    return trafos;
}

void PanoramaStitcher::writeTrafos(const fs::path& file, WriteType writeType)
{
    std::vector<cv::Mat> trafos;
    trafos.reserve(mImages.size());
    for (std::size_t i = 0; i < mImages.size(); i++)
        trafos.push_back(mOptimizedTrafos[i]);

    if (writeType == WriteType::Binary)
    {
        std::ofstream stream(file.string(), std::ios::out | std::ios::binary);
        checkStream(stream, file);
        {
            cereal::PortableBinaryOutputArchive archive(stream);
            archive(trafos);
        }
        return;
    }

    cv::FileStorage fs(file.string(), cv::FileStorage::WRITE);
    checkFileStorage(fs, file);

    auto rect = generateBoundingRect();
    rect.width = rect.width - rect.x;
    rect.height = rect.height - rect.y;

    fs << "bounding_rect" << rect;
    fs << "trafos" << trafos;
    fs.release();
}
} // namespace ht
