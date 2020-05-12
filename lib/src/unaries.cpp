#include "habitrack/unaries.h"

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <spdlog/spdlog.h>

#include "habitrack/images.h"
#include "habitrack/transformation.h"
#include "progressBar.h"

namespace ht
{
using namespace matches;
using namespace transformation;
namespace fs = std::filesystem;

Unaries Unaries::compute(const Images& imgContainer, const fs::path& unDir, std::size_t start,
    std::size_t end, bool removeLaser, double subsample, const matches::PairwiseTrafos& trafos,
    std::size_t cacheSize)
{
    // make sure folder exits
    if (!fs::exists(unDir) || !fs::is_directory(unDir))
        fs::create_directories(unDir);

    end = std::min(imgContainer.size(), end);

    auto numImgs = end - start;
    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    pairs.reserve(numImgs - 1);

    for (std::size_t i = start; i < end - 1; i++)
        pairs.push_back({i, i + 1});

    auto cache = imgContainer.getPairwiseCache(cacheSize, pairs);
    spdlog::info("Computing unaries");
    ProgressBar bar(cache.getNumChunks());
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
                auto trafo = invert(
                    trafos.at({currPair.first, currPair.second}), GeometricType::Homography);
                cv::Mat next_warped;
                cv::warpPerspective(next_gray, next_warped, trafo, next.size());
                diff = ref_gray - next_warped;
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
            double quality = getUnaryQuality(resizedUnary);
            spdlog::debug("Unary computed of image {} with quality {}", currPair.first, quality);
            unaries[k] = diff;
        }
        writeChunk(imgContainer, unDir, cache.getChunkBounds(i), unaries, start);
    }
    return Unaries();
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

void Unaries::writeUnary(const fs::path& file, const cv::Mat& unary)
{
    cv::imwrite(file.string(), unary);
    /* std::ofstream stream(file.string(), std::ios::out | std::ios::binary); */
    /* checkStream(stream, file); */
    /* { */
    /*     cereal::PortableBinaryOutputArchive archive(stream); */
    /*     archive(unary); */
    /* } */
}

double Unaries::getUnaryQuality(const cv::Mat& unary)
{
    cv::Scalar meanChannels = cv::mean(unary);
    double meanValue = meanChannels[0];
    double quality = meanValue / 255.0;
    return quality;
}

std::filesystem::path Unaries::getFileName(const std::filesystem::path& unDir,
    const std::filesystem::path& stem)
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
