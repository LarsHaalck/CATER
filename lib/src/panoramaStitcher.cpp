#include "habitrack/panoramaStitcher.h"

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/stitching.hpp>

#include <Eigen/Dense>
//#include <ceres/ceres.h>

#include "habitrack/isometry.h"
/* #include "io/imageReader.h" */
/* #include "io/featureReader.h" */
/* #include "io/matchesReader.h" */

#include "homographyGlobalOptimizer.h"
#include "affinityGlobalOptimizer.h"
#include "similarityGlobalOptimizer.h"
#include "isometryGlobalOptimizer.h"

using Gt = ht::GeometricType;
namespace ht
{
PanoramaStitcher::PanoramaStitcher(std::shared_ptr<ImageContainer> imgContainer,
    std::shared_ptr<FeatureContainer> ftContainer, std::shared_ptr<MatchesContainer> matchContainer,
    const std::vector<size_t>& keyFrames, GeometricType type, bool blend)
    /* , const cv::Mat& camMat, const cv::Mat& distCoeffs) */
    : mImgContainer(std::move(imgContainer))
    , mFtContainer(std::move(ftContainer))
    , mMatchContainer(std::move(matchContainer))
    , mKeyFrames(keyFrames)
    , mType(type)
    , mBlend(blend)
    /* , mCamMat(camMat) */
    /* , mCamMatInv() */
    /* , mDistCoeffs(distCoeffs) */
{
    /* if (mCamMat.empty()) */
    /* { */
    /*     mCamMat = cv::Mat::eye(3, 3, CV_64F); */
    /*     mCamMat.at<double>(0, 0) = 2304.0; */
    /*     mCamMat.at<double>(1, 1) = 2304.0; */
    /*     mCamMat.at<double>(0, 2) = 960.0; */
    /*     mCamMat.at<double>(1, 2) = 540.0; */

    /*     mDistCoeffs = cv::Mat::zeros(5, 1, CV_64F); */
    /* } */
    /* mDistCoeffs.at<double>(2, 0) = mDistCoeffs.at<double>(3, 0) = 0.0; */
    /* cv::invert(mCamMat, mCamMatInv); */

    /* auto size = mImgReader->getImage(0).size(); */
    /* Thinning thinning(mFtReader, mMatchReader, size.height, size.width); */
    /* mKeyFrames = thinning.estimate(0.3, 0.5, 50); */
    /* /1* mKeyFrames.erase(mKeyFrames.begin() + 28, mKeyFrames.end()); *1/ */
    /* std::cout << "Keeping: " << mKeyFrames.size() << " of " << mImgReader->numImages()  << std::endl; */
    /* mKeyFrames = {0, 5, 10, 15, 20, 26, 29, 36}; */
}


/* cv::Mat PanoGenerator::draw(const cv::Mat& trafo, size_t idI, size_t idJ) */
/* { */
/*     auto imgJ = mImgReader->getImage(idI); */
/*     auto trafoTriple = getTransformation(idI, idJ); */
/*     auto ptsI = std::get<1>(trafoTriple); */
/*     auto ptsJ = std::get<2>(trafoTriple); */

/*     std::vector<cv::Point2f> ptsITrans; */
/*     cv::perspectiveTransform(ptsI, ptsITrans, trafo); */

/*     for (size_t i = 0; i < ptsI.size(); i++) */
/*     { */
/*         cv::drawMarker(imgJ, ptsJ[i], cv::Scalar(0, 255, 0)); */
/*         cv::drawMarker(imgJ, ptsITrans[i], cv::Scalar(0, 0, 255)); */
/*         cv::line(imgJ, ptsJ[i], ptsITrans[i], cv::Scalar(0, 255, 255)); */
/*     } */

/*     cv::putText(imgJ, std::to_string(idI) + " to " + std::to_string(idJ), */
/*         cv::Point(200, 200), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0)); */


/*     cv::imshow("bla", imgJ); */
/*     while (cv::waitKey(0) != 27) */
/*     { */
/*     } */

/*     return imgJ; */
/* } */

/* cv::Mat interpolateHomography(double alpha, const cv::Mat& homography) */
/* { */
/*     cv::Mat eye = cv::Mat::eye(3, 3, CV_64F); */
/*     return  alpha * homography + (1.0 - alpha) * eye; */
/* } */

/* std::vector<size_t> getMatchedImgs(size_t currImg, const std::vector<std::pair<size_t, size_t>>& matchPairs) */
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

/* void PanoGenerator::addFunctor(ceres::Problem& problem, const cv::Point2f& ptI, */
/*     const cv::Point2f& ptJ, cv::Mat* trafoI, cv::Mat* trafoJ, */
/*     double* camParams, double* distParams, std::vector<double>* paramsI, */
/*     std::vector<double>* paramsJ) */
/* { */
/*     switch (mType) */
/*     { */
/*         case TrafoType::Isometry: */
/*         { */
/*             // Isometries need a parametrization vector because the angle needs to */
/*             // be optimized explicitly !!! */
/*             if (paramsI->empty()) */
/*             { */
/*                 double angle0 = std::atan2((*trafoI).at<double>(1, 0), */
/*                     (*trafoI).at<double>(0, 0)); */
/*                 (*paramsI)[0] = angle0; */
/*                 (*paramsI)[1] = (*trafoI).at<double>(0, 2); */
/*                 (*paramsI)[2] = (*trafoI).at<double>(1, 2); */
/*             } */
/*             if (paramsJ->empty()) */
/*             { */
/*                 double angle1 = std::atan2((*trafoJ).at<double>(1, 0), */
/*                     (*trafoJ).at<double>(0, 0)); */
/*                 (*paramsJ)[0] = angle1; */
/*                 (*paramsJ)[1] = (*trafoJ).at<double>(0, 2); */
/*                 (*paramsJ)[2] = (*trafoJ).at<double>(1, 2); */
/*             } */

/*             using Functor = IsometryGlobalFunctor; */
/*             Functor* functor = new Functor( */
/*                 Eigen::Vector2d(ptI.x, ptI.y), */
/*                 Eigen::Vector2d(ptJ.x, ptJ.y)); */

/*             problem.AddResidualBlock( */
/*                 new ceres::AutoDiffCostFunction<Functor, 4, 4, 3, 3, 3>(functor), */
/*                 new ceres::HuberLoss(1.0), camParams, distParams, */
/*                 (double*)paramsI->data(), (double*)paramsJ->data()); */

/*             problem.SetParameterLowerBound((double*)paramsI->data(), 0, 0); */
/*             problem.SetParameterUpperBound((double*)paramsI->data(), 0, 2 * 3.14159); */

/*             problem.SetParameterLowerBound((double*)paramsJ->data(), 0, 0); */
/*             problem.SetParameterUpperBound((double*)paramsJ->data(), 0, 2 * 3.14159); */
/*             break; */
/*         } */
/*         case TrafoType::Similarity: */
/*         { */
/*             using Functor = SimilarityGlobalFunctor; */
/*             Functor* functor = new Functor( */
/*                 Eigen::Vector2d(ptI.x, ptI.y), */
/*                 Eigen::Vector2d(ptJ.x, ptJ.y)); */

/*             // use all first 6 elements although only 4 in total are needed */
/*             // to avoid copying data and use "repairTrafos" to update ignored values */
/*             problem.AddResidualBlock( */
/*                 new ceres::AutoDiffCostFunction<Functor, 4, 4, 3, 6, 6>(functor), */
/*                 new ceres::HuberLoss(1.0), camParams, distParams, */
/*                 (double*)trafoI->data, (double*)trafoJ->data); */
/*             break; */
/*         } */
/*         case TrafoType::Affinity: */
/*         { */
/*             using Functor = AffinityGlobalFunctor; */
/*             Functor* functor = new Functor( */
/*                 Eigen::Vector2d(ptI.x, ptI.y), */
/*                 Eigen::Vector2d(ptJ.x, ptJ.y)); */

/*             problem.AddResidualBlock( */
/*                 new ceres::AutoDiffCostFunction<Functor, 4, 4, 3, 6, 6>(functor), */
/*                 new ceres::HuberLoss(1.0), camParams, distParams, */
/*                 (double*)trafoI->data, (double*)trafoJ->data); */
/*             break; */
/*         } */
/*         case TrafoType::Homography: */
/*         { */
/*             using Functor = HomographyGlobalFunctor; */
/*             Functor* functor = new Functor( */
/*                 Eigen::Vector2d(ptI.x, ptI.y), */
/*                 Eigen::Vector2d(ptJ.x, ptJ.y)); */

/*             problem.AddResidualBlock( */
/*                 new ceres::AutoDiffCostFunction<Functor, 4, 4, 3, 9, 9>(functor), */
/*                 new ceres::HuberLoss(1.0), camParams, distParams, */
/*                 (double*)trafoI->data, (double*)trafoJ->data); */
/*             break; */
/*         } */
/*         default: */
/*             break; */
/*     } */
/* } */

/* void PanoGenerator::repairTrafos(std::vector<cv::Mat>& trafos, */
/*     const std::vector<std::vector<double>>& params) */
/* { */
/*     switch(mType) */
/*     { */
/*         case TrafoType::Isometry: */
/*             // rebuild matrix from params vector */
/*             for (size_t i = 0; i < trafos.size(); i++) */
/*             { */
/*                 trafos[i].at<double>(0, 0) = std::cos(params[i][0]); */
/*                 trafos[i].at<double>(0, 1) = std::sin(params[i][0]); */
/*                 trafos[i].at<double>(1, 0) = -std::sin(params[i][0]); */
/*                 trafos[i].at<double>(1, 1) = std::cos(params[i][0]); */

/*                 trafos[i].at<double>(0, 2) = params[i][1]; */
/*                 trafos[i].at<double>(1, 2) = params[i][2]; */
/*             } */
/*             break; */
/*         case TrafoType::Similarity: */
/*             // ensure similartiy property */
/*             for (auto& trafo : trafos) */
/*             { */
/*                 trafo.at<double>(1, 0) = -trafo.at<double>(0, 1); */
/*                 trafo.at<double>(1, 1) = trafo.at<double>(0, 0); */
/*             } */
/*             break; */
/*         case TrafoType::Affinity: */
/*             break; */
/*         case TrafoType::Homography: */
/*             break; */
/*         default: */
/*             break; */
/*     } */
/* } */

/* std::vector<double> PanoGenerator::getCamParameterization() */
/* { */
/*     std::vector<double> params; */
/*     params.reserve(4); */

/*     // fx, fy */
/*     params.push_back(mCamMat.at<double>(0, 0)); */
/*     params.push_back(mCamMat.at<double>(1, 1)); */

/*     // px,  py */
/*     params.push_back(mCamMat.at<double>(0, 2)); */
/*     params.push_back(mCamMat.at<double>(1, 2)); */
/*     return params; */
/* } */

/* std::vector<double> PanoGenerator::getDistParameterization() */
/* { */
/*     std::vector<double> params; */
/*     params.reserve(3); */

/*     // skip tangential coeffs */
/*     params.push_back(mDistCoeffs.at<double>(0, 0)); */
/*     params.push_back(mDistCoeffs.at<double>(1, 0)); */
/*     params.push_back(mDistCoeffs.at<double>(4, 0)); */
/*     return params; */
/* } */

/* std::vector<double> PanoGenerator::invertDistParameterization( */
/*     const std::vector<double>& params) */
/* { */
/*     std::vector<double> invParams; */
/*     if (params.empty()) */
/*         return invParams; */

/*     invParams.reserve(3); */
/*     invParams.push_back(-params[0]); */
/*     invParams.push_back(3 * params[0] * params[0] - params[1]); */
/*     invParams.push_back(8 * params[0] * params[1] - 12 * params[0] * params[0] * params[0] - params[2]); */
/*     return invParams; */
/* } */

/* void PanoGenerator::repairIntriniscs(const std::vector<double>& camParams, */
/*     const std::vector<double>& distParams) */
/* { */
/*     mCamMat.at<double>(0, 0) = camParams[0]; */
/*     mCamMat.at<double>(1, 1) = camParams[1]; */
/*     mCamMat.at<double>(0, 2) = camParams[2]; */
/*     mCamMat.at<double>(1, 2) = camParams[3]; */
/*     cv::invert(mCamMat, mCamMatInv); */

/*     auto invertedDistParams = invertDistParameterization(distParams); */
/*     mDistCoeffs.at<double>(0, 0) = invertedDistParams[0]; */
/*     mDistCoeffs.at<double>(1, 0) = invertedDistParams[1]; */
/*     mDistCoeffs.at<double>(4, 0) = invertedDistParams[4]; */
/* } */

/* cv::Mat PanoGenerator::optimizeGlobal() */
/* { */
/*     auto matchPairs = mMatchReader->getMatchPairs(); */
/*     std::sort(std::begin(matchPairs), std::end(matchPairs)); */

/*     auto camParams = getCamParameterization(); */
/*     auto distParams = getDistParameterization(); */

/*     // invert params to use the model as an undistortion model instead of an distortion */
/*     // model */
/*     distParams = invertDistParameterization(distParams); */

/*     // store trafos relative to first frame */
/*     /1* std::vector<cv::Mat> globalTrafos; *1/ */
/*     globalTrafos.reserve(mKeyFrames.size()); */
/*     globalTrafos.push_back(cv::Mat::eye(3, 3, CV_64F)); */

/*     std::vector<std::vector<double>> params; */
/*     if (mType == TrafoType::Isometry) */
/*     { */
/*         params = std::vector<std::vector<double>>(mKeyFrames.size(), */
/*             std::vector<double>(3)); */
/*     } */

/*     ceres::Problem problem; */
/*     ceres::Solver::Options options; */
/*     options.num_threads = 16; */
/*     options.max_num_iterations = 500; */
/*     options.minimizer_progress_to_stdout = true; */

/*     size_t lastBA = -1; */
/*     for(size_t i = 1; i < mKeyFrames.size(); i++) */
/*     { */
/*         size_t currImg = mKeyFrames[i]; */
/*         size_t prevImg = mKeyFrames[i - 1]; */

/*         /1* // get trafo to previous image *1/ */
/*         auto trafo = std::get<0>(getTransformation(prevImg, currImg)); */

/*         // chain transforamtion with previous to get trafo relative to first frame */
/*         trafo = invertSpecial(trafo, mType); */
/*         trafo = globalTrafos[globalTrafos.size() - 1] * trafo; */
/*         globalTrafos.push_back(trafo); */

/*         bool doBA = false; */
/*         auto currMatchedIds = getMatchedImgs(currImg, matchPairs); */
/*         for (auto id : currMatchedIds) */
/*         { */
/*             auto trafoTriple = getTransformation(id, currImg); */
/*             if (std::get<0>(trafoTriple).empty()) */
/*                 continue; */

/*             if (id + 20 < currImg) */
/*                 doBA = true; */
/*             auto ptsI = std::get<1>(trafoTriple); */
/*             auto ptsJ = std::get<2>(trafoTriple); */

/*             for (size_t k = 0; k < ptsI.size(); k++) */
/*             { */
/*                 cv::Mat* trafoI = &globalTrafos[id]; */
/*                 cv::Mat* trafoJ = &globalTrafos[currImg]; */
/*                 addFunctor(problem, ptsI[k], ptsJ[k], trafoI, trafoJ, camParams.data(), */
/*                     distParams.data(), &params[id], &params[currImg]); */
/*                 /1* problem.SetParameterBlockConstant((double*)trafoI->data); *1/ */
/*                 /1* problem.SetParameterBlockConstant((double*)trafoJ->data); *1/ */
/*             } */
/*         } */
/*         if (true && (doBA && lastBA + 20 < currImg)) */
/*         { */
/*             problem.SetParameterBlockConstant((double*)globalTrafos[0].data); */
/*             problem.SetParameterBlockConstant(camParams.data()); */
/*             problem.SetParameterBlockConstant(distParams.data()); */

/*             lastBA = currImg; */
/*             std::cout << "Doing Bundle Adjustment at frame: " << currImg << std::endl; */
/*             ceres::Solver::Summary summary; */
/*             ceres::Solve(options, &problem, &summary); */
/*             repairTrafos(globalTrafos, params); */
/*             repairIntriniscs(camParams, distParams); */
/*             std::cout << summary.FullReport() << std::endl; */
/*             doBA = false; */
/*         } */
/*     } */


/*     /1* problem.SetParameterBlockVariable(camParams.data()); *1/ */
/*     /1* problem.SetParameterBlockVariable(distParams.data()); *1/ */

/*     std::cout << mCamMat << std::endl; */
/*     problem.SetParameterBlockConstant((double*)globalTrafos[0].data); */
/*     problem.SetParameterBlockConstant(camParams.data()); */
/*     problem.SetParameterBlockConstant(distParams.data()); */
/*     ceres::Solver::Summary summary; */
/*     ceres::Solve(options, &problem, &summary); */
/*     std::cout << summary.FullReport() << std::endl; */

/*     repairTrafos(globalTrafos, params); */
/*     repairIntriniscs(camParams, distParams); */

/*     std::cout << mCamMat << std::endl; */
/*     std::cout << mDistCoeffs << std::endl; */

/*     auto imgSize = mImgReader->getImage(mKeyFrames[0]).size(); */
/*     auto boundingRect = generateBoundingRect(globalTrafos, imgSize); */
/*     cv::Size targetSize(2560, 1440); */
/*     return std::get<0>(stitchPano(boundingRect, globalTrafos, targetSize)); */
/* } */




/* //TODO connected components */
/* std::tuple<cv::Mat, cv::Mat, cv::Mat> PanoGenerator::generatePano(cv::Size targetSize) */
/* { */
/*     if (mKeyFrames.empty()) */
/*         return std::make_tuple(cv::Mat(), cv::Mat(), cv::Mat()); */

/*     auto img0Size = mImgReader->getImage(mKeyFrames[0]).size(); */
/*     cv::Rect2d boundingRect(0, 0, img0Size.width, img0Size.height); */
/*     /1* std::cout << boundingRect << std::endl; *1/ */

/*     // store trafos relative to first frame */
/*     std::vector<cv::Mat> trafos; */
/*     trafos.reserve(mKeyFrames.size()); */
/*     trafos.push_back(cv::Mat::eye(3, 3, CV_64F)); */

/*     for(size_t i = 1; i < mKeyFrames.size(); i++) */
/*     { */
/*         size_t currImg = mKeyFrames[i]; */
/*         size_t prevImg = mKeyFrames[i - 1]; */

/*         // get image size */
/*         auto imgSize = mImgReader->getImage(currImg).size(); */

/*         // get trafo to previous image */
/*         auto trafoTriple = getTransformation(prevImg, currImg); */
/*         /1* trafos.push_back(std::get<0>(trafoTriple)); *1/ */

/*         auto trafo = std::get<0>(trafoTriple); */
/*         /1* std::cout << "\n" << trafo << "\n" << std::endl; *1/ */
/*         if (trafo.empty()) */
/*         { */
/*             std::cout << "Requested pano [" << mKeyFrames[0] << ", " */
/*                 << mKeyFrames[mKeyFrames.size() - 1] << "]" << std::endl; */
/*             std::cout << "Did pano [" << mKeyFrames[0] << ", " << prevImg << "]" */
/*                 << std::endl; */
/*             break; */
/*         } */
/*         trafo = invertSpecial(trafo, mType); */
/*         trafo = trafos[trafos.size() - 1] * trafo; */
/*         /1* std::cout << trafo << std::endl; *1/ */

/*         // chain transforamtion with previous to get trafo relative to first frame */
/*         trafos.push_back(trafo); */

/*         // update global boundingg rect */
/*         boundingRect = getBoundingRect(trafo, imgSize, boundingRect); */
/*         /1* std::cout << boundingRect << std::endl; *1/ */
/*     } */

/*     return stitchPano(boundingRect, trafos, targetSize); */
/* } */


/* std::tuple<cv::Mat, cv::Mat, cv::Mat> PanoGenerator::stitchPano(cv::Rect2d rect, */
/*     const std::vector<cv::Mat>& trafos, */
/*     cv::Size targetSize) */
/* { */
/*     rect.width = rect.width - rect.x; */
/*     rect.height = rect.height - rect.y; */

/*     double scale = 1.0; */
/*     if (rect.width > targetSize.width) */
/*         scale = std::min(targetSize.width / rect.width, scale); */
/*     if (rect.height > targetSize.height) */
/*         scale = std::min(targetSize.height / rect.height, scale); */

/*     cv::Mat scaleMat = getScaleMat(scale); */
/*     cv::Mat transMat = getTranslationMat(-rect.x, -rect.y); */
/*     cv::Size newSize(scale * rect.width, scale * rect.height); */
/*     std::cout << scaleMat << std::endl; */
/*     std::cout << newSize << std::endl; */

/*     cv::detail::MultiBandBlender blender(false, 1); */
/*     blender.prepare(cv::Rect(0, 0, newSize.width, newSize.height)); */

/*     // here we start from 0 instead of 1 because we explicitly added the identity trafo */
/*     cv::Mat img0; */
/*     for(size_t i = 0; i < mKeyFrames.size(); i++) */
/*     { */
/*         cv::Mat currImg = mImgReader->getImage(mKeyFrames[i]); */
/*         cv::Mat mask(currImg.size(), CV_8UC1, cv::Scalar(255)); */

/*         // undistort iamge */
/*         cv::Mat undistImg; */
/*         cv::Mat undistMask; */
/*         /1* cv::undistort(currImg, undistImg, mCamMat, mDistCoeffs, mCamMat); *1/ */

/*         /1* cv::Mat newCam = cv::getOptimalNewCameraMatrix(mCamMat, mDistCoeffs, *1/ */
/*         /1*     currImg.size(), 0); *1/ */
/*         /1* std::cout << newCam << std::endl; *1/ */
/*         cv::Mat newCam = mCamMat; */
/*         cv::undistort(currImg, undistImg, mCamMat, mDistCoeffs, newCam); */
/*         cv::undistort(mask, undistMask, mCamMat, mDistCoeffs, newCam); */
/*         /1* newCam = mCamMat; *1/ */
/*         /1* undistImg = currImg; *1/ */


/*         if (i >= trafos.size()) */
/*             break; */

/*         auto currTrafo = trafos[i]; */
/*         cv::Mat newCamInv; */
/*         cv::invert(newCam, newCamInv); */
/*         currTrafo = mCamMat * trafos[i] * newCamInv; */
/*         cv::Mat warpedMask; */
/*         cv::warpPerspective(undistMask, warpedMask, scaleMat * transMat * currTrafo, newSize); */

/*         cv::Mat warped; */
/*         cv::warpPerspective(undistImg, warped, scaleMat * transMat  * currTrafo, newSize); */


/*         if (mBlend) */
/*         { */
/*             blender.feed(warped, warpedMask, cv::Point(0, 0)); */
/*         } */
/*         else */
/*         { */
/*             if (i == 0) */
/*                 img0 = undistImg; */
/*             warped.copyTo(img0, warpedMask); */

/*             /1* float scale = 1440.0 / std::max(img0.cols, img0.rows); *1/ */
/*             /1* cv::Mat resized; *1/ */
/*             /1* cv::resize(warpedMask, resized, cv::Size(0, 0), scale, scale); *1/ */
/*             /1* cv::namedWindow("bla", cv::WINDOW_NORMAL); *1/ */
/*             /1* cv::imshow("bla", resized); *1/ */
/*             /1* std::cout << mKeyFrames[i] << std::endl; *1/ */
/*             /1* while (cv::waitKey(0) != 27) *1/ */
/*             /1* { *1/ */

/*             /1* } *1/ */
/*         } */
/*     } */

/*     cv::Mat pano; */
/*     if (mBlend) */
/*     { */
/*         cv::Mat res, resMask; */
/*         blender.blend(res, resMask); */
/*         res.convertTo(pano, CV_8U); */
/*     } */
/*     else */
/*         pano = img0; */

/*     return std::make_tuple(pano, scaleMat, transMat); */
/* } */

/* std::tuple<cv::Mat, std::vector<cv::Point2f>, std::vector<cv::Point2f>> */
/* PanoGenerator::getTransformation(size_t idI, size_t idJ) */
/* { */
/*     auto emptyTuple = std::make_tuple(cv::Mat(), std::vector<cv::Point2f>(), */
/*         std::vector<cv::Point2f>()); */

/*     /1* std::cout << "getMatches: " << idI << "/ " << idJ << std::endl; *1/ */
/*     auto matches = mMatchReader->getMatches(idI, idJ); */
/*     if (matches.empty()) */
/*         return emptyTuple; */

/*     auto ftI = mFtReader->getFeatures(idI); */
/*     auto ftJ = mFtReader->getFeatures(idJ); */

/*     std::vector<cv::KeyPoint> ftIF, ftJF; */
/*     for (const auto& match : matches) */
/*     { */
/*         ftIF.push_back(ftI[match.queryIdx]); */
/*         ftJF.push_back(ftJ[match.trainIdx]); */
/*     } */

/*     std::vector<cv::Point2f> ptI; */
/*     for (const auto& elem : ftIF) */
/*         ptI.push_back(elem.pt); */

/*     std::vector<cv::Point2f> ptJ; */
/*     for (const auto& elem : ftJF) */
/*         ptJ.push_back(elem.pt); */

/*     std::vector<cv::Point2f> ptIUndist, ptJUndist; */
/*     cv::undistortPoints(ptI, ptIUndist, mCamMat, mDistCoeffs); */
/*     cv::undistortPoints(ptJ, ptJUndist, mCamMat, mDistCoeffs); */

/*     cv::Mat trafo; */
/*     std::vector<uchar> mask(ptI.size()); */
/*     switch(mType) */
/*     { */
/*         case TrafoType::Isometry: */
/*             trafo = cv::estimateIsometry2D(ptIUndist, ptJUndist, mask); */
/*             break; */
/*         case TrafoType::Similarity: */
/*             trafo = cv::estimateAffinePartial2D(ptIUndist, ptJUndist, mask); */
/*             break; */
/*         case TrafoType::Affinity: */
/*             trafo = cv::estimateAffine2D(ptIUndist, ptJUndist, mask); */
/*             break; */
/*         case TrafoType::Homography: */
/*             trafo = cv::findHomography(ptIUndist, ptJUndist, mask, cv::RANSAC); */
/*             break; */
/*     } */

/*     std::vector<cv::Point2f> ptIFiltered, ptJFiltered; */
/*     for (size_t r = 0; r < mask.size(); r++) */
/*     { */
/*         if (mask[r]) */
/*         { */
/*             ptIFiltered.push_back(ptI[r]); */
/*             ptJFiltered.push_back(ptJ[r]); */
/*         } */

/*     } */

/*     return std::make_tuple(trafo, ptIFiltered, ptJFiltered); */
/* } */

cv::Rect2d PanoramaStitcher::generateBoundingRect(const std::vector<cv::Mat>& trafos,
    const cv::Size& imgSize)
{
    cv::Rect2d boundingRect(0, 0, imgSize.width, imgSize.height);

    // store trafos relative to first frame
    for(size_t i = 1; i < mKeyFrames.size(); i++)
        boundingRect = generateBoundingRectHelper(trafos[i], imgSize, boundingRect);
    return boundingRect;
}

cv::Rect2d PanoramaStitcher::generateBoundingRectHelper(const cv::Mat& trafo, cv::Size size,
    cv::Rect2d currRect)
{
    cv::Mat corners = (cv::Mat_<double>(4, 2) <<
        0.0, 0.0,
        size.width - 1, 0.0,
        0.0, size.height - 1,
        size.width - 1, size.height - 1
    );

    cv::Mat cornersHomo;
    cv::convertPointsToHomogeneous(corners, cornersHomo);
    /* cv::Mat currTrafo = mCamMat * trafo * mCamMatInv; */
    cv::Mat currTrafo = trafo;

    for (int i = 0; i < cornersHomo.rows; i++)
    {
        auto vec = cornersHomo.at<cv::Vec3d>(i, 0);
        cv::Mat vecMat = currTrafo * cv::Mat(vec);
        cornersHomo.at<cv::Vec3d>(i, 0) = vecMat;
    }

    cv::Mat cornersTrafo;
    cv::convertPointsFromHomogeneous(cornersHomo, cornersTrafo);

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
} // namespace ht
