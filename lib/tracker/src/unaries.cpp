#include "tracker/unaries.h"

#include "image-processing/images.h"
#include "image-processing/transformation.h"
#include "image-processing/util.h"
#include "progressbar/progressBar.h"
#include "unaryIO.h"
#include <algorithm>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

namespace ht
{
using namespace matches;
using namespace transformation;
namespace fs = std::filesystem;

Unaries Unaries::compute(const Images& imgContainer, const fs::path& unDir, std::size_t start,
    std::size_t end, bool removeLaser, double subsample, double sigma, const PairwiseTrafos& trafos,
    std::size_t cacheSize, std::shared_ptr<BaseProgressBar> cb)
{
    // make sure folder exits
    if (!fs::exists(unDir) || !fs::is_directory(unDir))
        fs::create_directories(unDir);

    end = std::min(imgContainer.size(), end);

    auto numImgs = end - start;

    // get pairs for consecutive frames
    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    pairs.reserve(numImgs - 1);
    for (std::size_t i = start; i < end - 1; i++)
        pairs.push_back({i, i + 1});

    auto cache = imgContainer.getPairwiseCache(cacheSize, pairs);
    spdlog::info("Computing unaries");

    if (!cb)
        cb = std::make_shared<ProgressBar>();
    cb->status("Computing Unaries");
    cb->setTotal(cache.getNumChunks());
    for (std::size_t i = 0; i < cache.getNumChunks(); i++)
    {
        auto chunk = cache.getChunk(i);
        auto lower = cache.getChunkBounds(i).first;
        spdlog::debug("Got chunk {}/{} with size {}", i + 1, cache.getNumChunks(), chunk.size());

        std::vector<cv::Mat> unaries(cache.getChunkSize(i));
#pragma omp parallel for
        for (std::size_t k = 0; k < cache.getChunkSize(i); k++)
        {
            auto currPair = pairs[lower + k];
            auto ref = chunk[currPair.first];
            auto next = chunk[currPair.second];
            cv::Mat ref_gray, next_gray;
            cv::cvtColor(ref, ref_gray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(next, next_gray, cv::COLOR_BGR2GRAY);

            cv::Mat diff;
            if (!trafos.empty())
            {
                if (trafos.count(currPair))
                {
                    auto trafo = invert(trafos.at(currPair), GeometricType::Homography);

                    cv::Mat next_mask(next.size(), CV_8UC1, cv::Scalar(255));
                    cv::Mat next_warped, next_warped_mask;
                    cv::warpPerspective(next_gray, next_warped, trafo, next.size());
                    cv::warpPerspective(next_mask, next_warped_mask, trafo, next.size());

                    diff = ref_gray - next_warped;
                    cv::bitwise_and(diff, next_warped_mask, diff);
                }
                else
                    diff = cv::Mat::zeros(ref_gray.size(), ref_gray.type());

            }
            else
                diff = ref_gray - next_gray;

            if (removeLaser)
            {
                cv::Mat o1ref = getOOPO1(ref);
                cv::Mat o1next = getOOPO1(next);
                cv::Mat o1 = o1ref + o1next;
                diff = diff - o1;
            }
            cv::Mat resizedUnary;
            cv::resize(diff, resizedUnary, cv::Size(), subsample, subsample, cv::INTER_LINEAR);
            spdlog::debug("Unary computed of image {}", currPair.first);
            unaries[k] = resizedUnary;
        }
        writeChunk(imgContainer, unDir, cache.getChunkBounds(i), unaries, start);
        cb->inc();
    }
    cb->done();
    writeProperties(unDir, subsample, sigma, !trafos.empty());

    // sanity check
    if (isComputed(imgContainer, unDir, start, end))
        return Unaries::fromDir(imgContainer, unDir, start, end);

    spdlog::warn("Unaries were not computed sucessfully");
    return Unaries {};
}

bool Unaries::isComputed(
    const Images& imgContainer, const fs::path& unDir, std::size_t start, std::size_t end)
{
    end = std::min(imgContainer.size(), end);

    bool isComputed = true;
    // check if some file is missing or other type expected
    for (std::size_t i = start; i < end - 1; i++)
    {
        // only check for feature or descriptor because they are calculated together
        auto stem = imgContainer.getFileName(i).stem();
        auto unFile = getFileName(unDir, stem);
        if (!fs::is_regular_file(unFile))
        {
            spdlog::debug("Missing unary file: {}", unFile.string());
            isComputed = false;
            break;
        }
    }
    isComputed &= fs::is_regular_file(unDir / "unaries.json");
    return isComputed;
}

Unaries Unaries::fromDir(
    const Images& imgContainer, const fs::path& unDir, std::size_t start, std::size_t end)
{
    spdlog::info("Reading Unaries from disk");
    std::unordered_map<std::size_t, fs::path> files;
    end = std::min(imgContainer.size(), end);
    files.reserve(end - start);

    for (std::size_t i = start; i < end - 1; i++)
        files.insert({i, getFileName(unDir, imgContainer.getFileName(i).stem())});

    auto [subsample, sigma, removeCameraMotion] = readProperties(unDir);

    spdlog::debug("{} unary files with subsample {} available in folder {}", files.size(),
        subsample, unDir.string());

    auto center = imgContainer.getCenter();
    auto size = imgContainer.getImgSize();
    cv::Mat gaussian = util::scaledGauss2D(center.x, center.y, sigma, sigma, 1.0, size);
    cv::resize(gaussian, gaussian, cv::Size(), subsample, subsample, cv::INTER_LINEAR);
    return Unaries {unDir, files, size, subsample, sigma, removeCameraMotion, gaussian};
}

cv::Mat Unaries::at(std::size_t idx) const
{
    assert(mUnFiles.count(idx) && "idx out of range in Unaries::at()");

    auto file = mUnFiles.at(idx);
    cv::Mat unary = cv::imread(file.string(), cv::IMREAD_UNCHANGED);
    unary.convertTo(unary, CV_32FC1);
    if (mRemoveCamMotion)
        cv::multiply(unary, mGaussian, unary);

    // unary is saved as 8-Bit image
    unary /= 255.0f;
    unary += 0.0001f;
    return unary;
}

cv::Mat Unaries::previewAt(std::size_t idx) const
{
    assert(mUnFiles.count(idx) && "idx out of range in Unaries::previewAt()");

    auto file = mUnFiles.at(idx);
    cv::Mat unary = cv::imread(file.string(), cv::IMREAD_UNCHANGED);
    unary.convertTo(unary, CV_32FC1);
    cv::multiply(unary, mGaussian, unary);
    unary.convertTo(unary, CV_8UC1);
    cv::resize(unary, unary, mImgSize);
    return unary;
}

bool Unaries::exists(std::size_t idx) const { return mUnFiles.count(idx) > 0; }

size_t_vec Unaries::getIDs() const
{
    size_t_vec vec(mUnFiles.size());
    std::transform(std::begin(mUnFiles), std::end(mUnFiles), std::begin(vec),
        [](auto pair) { return pair.first; });
    std::sort(std::begin(vec), std::end(vec));
    return vec;
}

std::tuple<double, double, bool> Unaries::readProperties(const fs::path& unDir)
{
    auto file = unDir / "unaries.json";
    std::ifstream stream(file.string(), std::ios::in);
    io::checkStream(stream, file);
    double subsample, sigma;
    bool removeCameraMotion;
    {
        cereal::JSONInputArchive archive(stream);
        archive(CEREAL_NVP(subsample), CEREAL_NVP(sigma), CEREAL_NVP(removeCameraMotion));
    }
    return {subsample, sigma, removeCameraMotion};
}

Unaries::Unaries(const fs::path& unDir, const std::unordered_map<std::size_t, fs::path> unFiles,
    cv::Size imgSize, double subsample, double sigma, bool removeCameraMotion,
    const cv::Mat& gaussian)
    : mUnDir(unDir)
    , mUnFiles(unFiles)
    , mImgSize(imgSize)
    , mSubsample(subsample)
    , mSigma(sigma)
    , mRemoveCamMotion(removeCameraMotion)
    , mGaussian(gaussian)
{
}

void Unaries::writeChunk(const Images& imgContainer, const fs::path& unDir,
    std::pair<std::size_t, std::size_t> bounds, const std::vector<cv::Mat>& unaries,
    std::size_t start)
{
    const auto [lower, upper] = bounds;
    for (std::size_t i = lower; i < upper; i++)
    {
        auto idx = start + i;
        auto stem = imgContainer.getFileName(idx).stem();
        auto unFileName = getFileName(unDir, stem);
        writeUnary(unFileName, unaries[i - lower]);
    }
}

void Unaries::writeProperties(
    const fs::path& unDir, double subsample, double sigma, bool removeCameraMotion)
{
    auto file = unDir / "unaries.json";
    std::ofstream stream(file.string(), std::ios::out);
    io::checkStream(stream, file);
    {
        cereal::JSONOutputArchive archive(stream);
        archive(CEREAL_NVP(subsample), CEREAL_NVP(sigma), CEREAL_NVP(removeCameraMotion));
    }
}

void Unaries::writeUnary(const fs::path& file, const cv::Mat& unary)
{
    cv::imwrite(file.string(), unary);
}

double Unaries::getUnaryQuality(const cv::Mat& unary)
{
    cv::Scalar meanChannels = cv::mean(unary);
    double meanValue = meanChannels[0];
    return meanValue;
}

fs::path Unaries::getFileName(const fs::path& unDir, const fs::path& stem)
{
    auto fullFile = unDir / stem;
    fullFile += "-unary.png";
    return fullFile;
}

cv::Mat Unaries::getOOPO1(const cv::Mat& img)
{
    cv::Mat channels[3];
    cv::split(img, channels);
    cv::Mat blue;
    channels[0].convertTo(blue, CV_32FC1);
    blue /= 255.;
    cv::Mat green;
    channels[1].convertTo(green, CV_32FC1);
    green /= 255.;
    cv::Mat red;
    channels[2].convertTo(red, CV_32FC1);
    red /= 255.;

    cv::Mat ret_o1;
    cv::Mat o1 = red - green;
    o1 /= sqrt(2.);
    o1 *= 255.;
    o1.convertTo(ret_o1, CV_8UC1);
    return ret_o1;
}

} // namespace ht
