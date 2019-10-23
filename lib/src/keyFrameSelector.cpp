#include "habitrack/keyFrameSelector.h"
#include "habitrack/featureContainer.h"
#include "habitrack/matchesContainer.h"

#include "progressBar.h"

#ifdef _OPENMP
   #include <omp.h>
#else
   #define omp_get_max_threads() 0
   #define omp_get_thread_num() 0
#endif // _OPENMP

namespace ht
{
KeyFrameSelector::KeyFrameSelector(std::shared_ptr<FeatureContainer> ftContainer)
    : mFtContainer(std::move(ftContainer))
    , mImgSize(mFtContainer->getImgSize())
    , mArea(mImgSize.area())
{
}

std::vector<std::size_t> KeyFrameSelector::compute(float relLow, float relHigh)
{
    const auto [low, high] = getRealLowHigh(relLow, relHigh);
    std::vector<std::size_t> keyFrames{};

    std::size_t currView = 0;
    keyFrames.push_back(currView);

    std::cout << "Selecting key frames..." << std::endl;
    ProgressBar bar(mFtContainer->getNumImgs());
    // find next keyframe until all framges have been processed
    while (currView < mFtContainer->getNumImgs())
    {
        std::size_t remainImgs = mFtContainer->getNumImgs() - currView - 1;
        std::vector<std::pair<float, std::size_t>> distOverlapVec;
        distOverlapVec.reserve(remainImgs);

        // control relative offset to current frame
        // thread 0 handles direct neighbor, thread 1 direct neighbor + 1, etc
        for (std::size_t k = 0; k < remainImgs; k += omp_get_max_threads())
        {
            std::vector<std::pair<float, std::size_t>> currDistOverlapVec(
                omp_get_max_threads());

            #pragma omp parallel
            {
                std::size_t currThread = omp_get_thread_num();
                std::size_t nextView = currView + 1 + (k + currThread);


                if (nextView < mFtContainer->getNumImgs())
                {
                    currDistOverlapVec[currThread] =
                        getMedianDistanceShift(currView, nextView);
                }
            }

            // investigate tuples, break if shift is to high or no feature points
            // for two consecutive frames
            bool warning = false;
            for (const auto& pair : currDistOverlapVec)
            {
                auto [shift, numFts] = pair;
                if (numFts == 0)
                {
                    if (warning)
                        break;
                    warning = true;
                }
                else
                    warning = false;

                if (shift > high)
                {
                    warning = true;
                    break;
                }
                distOverlapVec.push_back(pair);
            }
            if (warning)
                break;
        }

        // only happens at the end of all frames
        if (distOverlapVec.empty())
            break;

        std::size_t offset = filterViews(distOverlapVec, low, high);

        // + 1 because ids in overlap vec are relative to first neighbor
        currView += offset + 1;
        keyFrames.push_back(currView);
        bar += offset + 1;
        bar.display();
    }
    bar.done();

    std::cout << "Keeping: " << keyFrames.size() << " of " << mFtContainer->getNumImgs()
        << " files. " << std::endl;
    return keyFrames;
}

std::pair<float, float> KeyFrameSelector::getRealLowHigh(float low, float high) const
{
    // get image sizes (in this setup all images have the same width and height
    auto width = mImgSize.width;
    auto height = mImgSize.height;
    std::size_t minWH = std::min(width, height);
    float realLow = (low * minWH) * (low * minWH);
    float realHigh;
    if (high > 0)
        realHigh = (high * minWH) * (high * minWH);
    else // choose greatest possible value
        realHigh = (width * width) + (height * height);

    return std::make_pair(realLow, realHigh);
}

std::pair<float, std::size_t> KeyFrameSelector::getMedianDistanceShift(
    std::size_t idI, std::size_t idJ) const
{
    // TODO: store?????????
    auto matcher = std::make_unique<MatchesContainer>(mFtContainer,
        "", MatchType::Manual, 0, GeometricType::Homography);

    auto ftsI = mFtContainer->featureAt(idI);
    auto ftsJ = mFtContainer->featureAt(idJ);
    auto [trafos, matches] = matcher->compute(idI, idJ);

    // skip putative matches and empty trafo
    auto trafo = trafos[1];
    auto geomMatches = matches[1];

    std::vector<float> distances;
    std::vector<cv::Point2f> srcFiltered, dstFiltered;
    for (std::size_t i = 0; i < geomMatches.size(); i++)
    {
        auto ptI = ftsI[geomMatches[i].queryIdx].pt;
        auto ptJ = ftsJ[geomMatches[i].trainIdx].pt;

        distances.push_back(l2Dist(ptI, ptJ));
        srcFiltered.push_back(ptI);
        dstFiltered.push_back(ptJ);
    }

    /* double reprojError = calcReprojError(srcFiltered, dstFiltered, trafo); */
    if (srcFiltered.size() < 10)
        return std::make_pair(0.0f, 0);

    return std::make_pair(getMedian(distances), distances.size());

}

double KeyFrameSelector::calcReprojError(const std::vector<cv::Point2f>& ptsSrc,
    std::vector<cv::Point2f>& ptsDst, const cv::Mat& trafo) const
{
    cv::Mat transTrafo;
    if (trafo.rows == 2)
    {
        transTrafo = cv::Mat::eye(3, 3, CV_32F);
        trafo.copyTo(transTrafo.rowRange(0, 2));
    }
    else
        transTrafo = trafo;

    std::vector<cv::Point2f> transSrc;
    cv::perspectiveTransform(ptsSrc, transSrc, transTrafo);

    double error = 0;
    for (std::size_t i = 0; i < ptsSrc.size(); i++)
        error = l2Dist(transSrc[i], ptsDst[i]);

    return error / ptsSrc.size();
}

std::size_t KeyFrameSelector::filterViews(
    const std::vector<std::pair<float, std::size_t>>& distOverlapVec, float low,
    float high)
{
    auto maxView = std::max_element(std::begin(distOverlapVec), std::end(distOverlapVec),
        [&](const auto& lhs, const auto& rhs) {
            return compareMaxOverlap(lhs, rhs, low, high);
        });

    /* if (maxView == std::end(distOverlapVec)) */
    /*     std::cout << "should not happen" << std::endl; */

    return static_cast<std::size_t>(
        std::distance(std::begin(distOverlapVec), maxView));
}

bool KeyFrameSelector::compareMaxOverlap(const std::pair<float, std::size_t>& lhs,
    const std::pair<float, std::size_t>& rhs, float low, float high) const
{
    auto isInRange = [low, high](const auto& pair) -> bool {
        return ((pair.first >= low) && (pair.first <= high));
    };

    if (!isInRange(lhs) && !isInRange(rhs))
        return (lhs.first < rhs.first);
    if (!isInRange(lhs) && isInRange(rhs))
        return true;
    if (isInRange(lhs) && !isInRange(rhs))
        return false;
    if (isInRange(lhs) && isInRange(rhs))
        return (lhs.second < rhs.second);

    // only for compiler warning, can't happen
    return true;
}

} // namespace ht
