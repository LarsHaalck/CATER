#include <habitrack/panorama/keyFrames.h>

#include <habitrack/image-processing/util.h>
#include <habitrack/util/algorithm.h>
#include <habitrack/util/stopWatch.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
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
    const std::filesystem::path& file, float relLow, float relHigh, Strategy strategy,
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
        std::vector<detail::KeyFrameData> distOverlapVec;
        int extended = -1;

        // control relative offset to current frame
        // thread 0 handles direct neighbor, thread 1 direct neighbor + 1, etc
        for (std::size_t k = 0; k < remainImgs; k += omp_get_max_threads())
        {
            std::vector<detail::KeyFrameData> currDistOverlapVec(omp_get_max_threads());

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
            for (const auto& tuple : currDistOverlapVec)
            {
                auto shift = std::get<0>(tuple);
                auto numFts = std::get<1>(tuple);
                if (numFts == 0)
                {
                    if (extended < 0)
                        extended = 50;
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
                distOverlapVec.push_back(tuple);
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

        std::size_t offset = detail::filterViews(distOverlapVec, low, high, strategy);

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
        float realLow = (low * minWH);
        float realHigh;
        if (high > 0)
            realHigh = (high * minWH);
        else // choose greatest possible value
            realHigh = std::sqrt((width * width) + (height * height));

        return std::make_pair(realLow, realHigh);
    }

    KeyFrameData getMedianDistanceShift(
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
        float response = 0;
        for (std::size_t i = 0; i < geomMatches.size(); i++)
        {
            auto ftI = ftsI[geomMatches[i].queryIdx];
            auto ftJ = ftsJ[geomMatches[i].trainIdx];
            response += (ftI.response * ftI.response);
            auto ptI = ftI.pt;
            auto ptJ = ftJ.pt;

            distances.push_back(util::euclidianDist(ptI, ptJ));
            srcFiltered.push_back(ptI);
            dstFiltered.push_back(ptJ);
        }
        response /= geomMatches.size();

        if (srcFiltered.size() < 30)
            return std::make_tuple(0.0f, 0, 0.0f, 0.0f, 0.0f);

        // negative reproj err is maximized -> reproj error is minimized
        auto reprojError = -calcReprojError(srcFiltered, dstFiltered, trafo);
        auto coverage = 0.5
            * (cv::boundingRect(srcFiltered).area() + cv::boundingRect(dstFiltered).area())
            / fts.getImageSize().area();
        return std::make_tuple(median_fast(std::begin(distances), std::end(distances)),
            distances.size(), coverage, response, reprojError);
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
        std::vector<KeyFrameData> distOverlapVec, float low, float high, Strategy strategy)
    {
        // find first element from behind which has non zero num fts
        auto non_zero = std::find_if(std::rbegin(distOverlapVec), std::rend(distOverlapVec),
            [](const auto& p) { return std::get<1>(p) != 0; });
        // and erase until this frame
        distOverlapVec.erase(non_zero.base(), std::end(distOverlapVec));

        if (distOverlapVec.empty())
        {
            spdlog::warn(
                "Found no feature points, progressing by one frame only in keyframe selection");
            return 0;
        }

        auto counts = getBordaCounts(distOverlapVec, strategy);
        auto maxView = std::max_element(std::rbegin(counts), std::rend(counts),
            [&](const auto& lhs, const auto& rhs)
            { return compareMaxOverlap(lhs, rhs, low, high); });

        // find distance to reversed reverse iterator (+1 because of rev iterator implementation)
        return static_cast<std::size_t>(std::distance(std::begin(counts), (++maxView).base()));
    }

    std::vector<std::pair<float, std::size_t>> getBordaCounts(
        const std::vector<KeyFrameData>& data, Strategy strategy)
    {
        std::vector<std::pair<float, std::size_t>> res;
        res.reserve(data.size());
        if (strategy == Strategy::Matches)
        {
            // select only shift and num matches from data
            std::transform(std::begin(data), std::end(data), std::back_inserter(res),
                [](const auto& elem)
                { return std::make_pair(std::get<0>(elem), std::get<1>(elem)); });
        }
        else
        {
            auto borda1 = getBordaCount<1>(data);
            auto borda2 = getBordaCount<2>(data);
            auto borda3 = getBordaCount<3>(data);
            auto borda4 = getBordaCount<4>(data);
            for (std::size_t i = 0; i < data.size(); i++)
            {
                res.push_back(std::make_pair(
                    std::get<0>(data[i]), borda1[i] + borda2[i] + borda3[i] + borda4[i]));
            }
        }
        return res;
    }

    bool compareMaxOverlap(const std::pair<float, std::size_t>& lhs,
        const std::pair<float, std::size_t>& rhs, float low, float high)
    {
        auto isInRange = [low, high](const auto& pair) -> bool
        { return ((pair.first >= low) && (pair.first <= high)); };

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
