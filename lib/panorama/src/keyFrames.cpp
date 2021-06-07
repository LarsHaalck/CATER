#include "panorama/keyFrames.h"

#include "image-processing/util.h"
#include "util/algorithm.h"
#include "util/stopWatch.h"

#include <opencv2/core.hpp>
#include <spdlog/spdlog.h>

// stubs if omp is not available on the system
#ifdef _OPENMP
#include <omp.h>
#else
#define omp_get_max_threads() 1
#define omp_get_thread_num() 1
#endif // _OPENMP

namespace fs = std::filesystem;
namespace ht::KeyFrames
{
bool isComputed(const std::filesystem::path& file) { return fs::is_regular_file(file); }

std::vector<std::size_t> fromDir(const std::filesystem::path& file)
{
    spdlog::info("Reading Keyframes from disk");
    auto kf = detail::loadFromFile(file);

    spdlog::debug("Loaded keyframe file {} with size {}", file.string(), kf.size());
    return kf;
}

std::vector<std::size_t> compute(const BaseFeatureContainer& ftContainer, GeometricType type,
    const std::filesystem::path& file, float relLow, float relHigh,
    std::shared_ptr<BaseProgressBar> cb)
{
    const auto [low, high] = detail::getRealLowHigh(ftContainer.getImageSize(), relLow, relHigh);
    std::vector<std::size_t> keyFrames {};

    std::size_t currView = 0;
    keyFrames.push_back(currView);

    spdlog::info("Computing Keyframes");
    if (!cb)
        cb = std::make_shared<ProgressBar>();

    cb->status("Computing keyframes");
    cb->setTotal(ftContainer.size());
    PreciseStopWatch timer;

    // find next keyframe until all framges have been processed
    while (currView < ftContainer.size())
    {
        std::size_t remainImgs = ftContainer.size() - currView - 1;
        std::vector<std::pair<float, std::size_t>> distOverlapVec;
        int extended = -1;

        // control relative offset to current frame
        // thread 0 handles direct neighbor, thread 1 direct neighbor + 1, etc
        for (std::size_t k = 0; k < remainImgs; k += omp_get_max_threads())
        {
            std::vector<std::pair<float, std::size_t>> currDistOverlapVec(omp_get_max_threads());

#pragma omp parallel
            {
                std::size_t currThread = omp_get_thread_num();
                std::size_t nextView = currView + 1 + (k + currThread);

                if (nextView < ftContainer.size())
                {
                    currDistOverlapVec[currThread]
                        = detail::getMedianDistanceShift(currView, nextView, ftContainer, type);
                }
            }

            bool highShift = false;
            for (const auto& pair : currDistOverlapVec)
            {
                auto [shift, numFts] = pair;
                if (numFts == 0)
                {
                    if (extended < 0)
                        extended = 10;
                    else if (extended > 0)
                        extended--;
                    else
                        break;
                }
                if (shift > high)
                {
                    highShift = true;
                    break;
                }
                distOverlapVec.push_back(pair);
            }

            // we extended the window a bit and have not reached high shift yet
            if (extended > 0 && !highShift)
            {
                spdlog::debug("Window was extended for kf candiate {}", currView);
                continue;
            }

            // border case where velocity is too high
            if (highShift && distOverlapVec.empty())
                distOverlapVec.push_back(currDistOverlapVec[0]);

            // enough shift or no extension anymore --> break
            if (highShift || extended == 0)
                break;
        }

        // only happens at the end of all frames
        if (distOverlapVec.empty())
            break;

        std::size_t offset = detail::filterViews(distOverlapVec, low, high);

        // + 1 because ids in overlap vec are relative to first neighbor
        currView += offset + 1;
        keyFrames.push_back(currView);

        cb->inc(offset + 1);
    }
    // always add last frame, so we don't need to extrapolate in the reintergration step
    if (keyFrames[keyFrames.size() - 1] != ftContainer.size() - 1)
        keyFrames.push_back(ftContainer.size() - 1);

    auto elapsed_time = timer.elapsed_time<unsigned int, std::chrono::milliseconds>();
    cb->status("Finished");
    cb->done();
    spdlog::info("Keeping: {} / {} files", keyFrames.size(), ftContainer.size());
    spdlog::info("Computed keyframes in {} ms", elapsed_time);

    detail::writeToFile(file, keyFrames);
    return keyFrames;
}

namespace detail
{
    std::pair<float, float> getRealLowHigh(cv::Size imgSize, float low, float high)
    {
        // get image sizes (in this setup all images have the same width and height
        auto width = imgSize.width;
        auto height = imgSize.height;
        std::size_t minWH = std::min(width, height);
        float realLow = (low * minWH) * (low * minWH);
        float realHigh;
        if (high > 0)
            realHigh = (high * minWH) * (high * minWH);
        else // choose greatest possible value
            realHigh = (width * width) + (height * height);

        return std::make_pair(realLow, realHigh);
    }

    std::pair<float, std::size_t> getMedianDistanceShift(
        std::size_t idI, std::size_t idJ, const BaseFeatureContainer& fts, GeometricType type)
    {
        auto ftsI = fts.featureAt(idI);
        auto ftsJ = fts.featureAt(idJ);
        auto [trafos, matches] = matches::computePair(type, fts, idI, idJ);

        // skip putative matches and empty trafo
        auto trafo = trafos[1];
        auto geomMatches = matches[1];

        std::vector<float> distances;
        std::vector<cv::Point2f> srcFiltered, dstFiltered;
        for (std::size_t i = 0; i < geomMatches.size(); i++)
        {
            auto ptI = ftsI[geomMatches[i].queryIdx].pt;
            auto ptJ = ftsJ[geomMatches[i].trainIdx].pt;

            distances.push_back(util::euclidianDist(ptI, ptJ));
            srcFiltered.push_back(ptI);
            dstFiltered.push_back(ptJ);
        }

        /* double reprojError = calcReprojError(srcFiltered, dstFiltered, trafo); */
        if (srcFiltered.size() < 30)
            return std::make_pair(0.0f, 0);

        return std::make_pair(
            median_fast(std::begin(distances), std::end(distances)), distances.size());
    }

    double calcReprojError(const std::vector<cv::Point2f>& ptsSrc, std::vector<cv::Point2f>& ptsDst,
        const cv::Mat& trafo)
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
            error += util::euclidianDist(transSrc[i], ptsDst[i]);

        return error / ptsSrc.size();
    }

    std::size_t filterViews(
        std::vector<std::pair<float, std::size_t>> distOverlapVec, float low, float high)
    {
        // find first element from behind which has non zero num fts
        auto non_zero = std::find_if(std::rbegin(distOverlapVec), std::rend(distOverlapVec),
            [](const auto& p) { return p.second != 0; });
        // and erase until this frame
        distOverlapVec.erase(non_zero.base(), std::end(distOverlapVec));

        auto maxView = std::max_element(std::rbegin(distOverlapVec), std::rend(distOverlapVec),
            [&](const auto& lhs, const auto& rhs) {
                return compareMaxOverlap(lhs, rhs, low, high);
            });

        // find distance to reversed reverse iterator (+1 because of rev iterator implementation)
        return static_cast<std::size_t>(
            std::distance(std::begin(distOverlapVec), (++maxView).base()));
    }

    bool compareMaxOverlap(const std::pair<float, std::size_t>& lhs,
        const std::pair<float, std::size_t>& rhs, float low, float high)
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

    void writeToFile(const fs::path& file, const std::vector<std::size_t>& keyFrames)
    {
        cv::FileStorage fs(file.string(), cv::FileStorage::WRITE);
        if (!fs.isOpened())
        {
            throw std::filesystem::filesystem_error(
                "Error opening key frame file", file, std::make_error_code(std::errc::io_error));
        }

        std::vector<int> keyFramesInt(std::begin(keyFrames), std::end(keyFrames));
        fs << "key_frames" << keyFramesInt;
        fs.release();
    }

    std::vector<std::size_t> loadFromFile(const fs::path& file)
    {
        cv::FileStorage fs(file.string(), cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            throw std::filesystem::filesystem_error(
                "Error opening key frame file", file, std::make_error_code(std::errc::io_error));
        }

        std::vector<int> keyFramesInt;
        fs["key_frames"] >> keyFramesInt;

        std::vector<std::size_t> keyFrames(std::begin(keyFramesInt), std::end(keyFramesInt));
        return keyFrames;
    }
} // namespace detail
} // namespace ht
