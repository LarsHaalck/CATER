#ifndef TRACKER_UNARIES_H
#define TRACKER_UNARIES_H

#include "image-processing/matches.h"
#include "image-processing/types.h"
#include <filesystem>
#include <vector>

namespace ht
{
class Images;
}

namespace ht
{
class Unaries
{
public:
    Unaries() = default;

    static Unaries compute(const Images& imgContainer, const std::filesystem::path& unDir,
        std::size_t start = 0, std::size_t end = -1, bool removeLaser = false,
        double subsample = 0.8, double sigma = 200.0, const PairwiseTrafos& trafos = {},
        std::size_t cacheSize = 0, std::shared_ptr<BaseProgressBar> cb = {});

    static bool isComputed(const Images& imgContainer, const std::filesystem::path& unDir,
        std::size_t start = 0, std::size_t end = -1);
    static Unaries fromDir(const Images& imgContainer, const std::filesystem::path& unDir,
        std::size_t start = 0, std::size_t end = -1);

    cv::Mat at(std::size_t idx) const;
    cv::Mat previewAt(std::size_t idx) const;
    bool exists(std::size_t idx) const;
    size_t_vec getIDs() const;
    std::size_t size() const { return mUnFiles.size(); }

    static double getUnaryQuality(const cv::Mat& unary);

    std::filesystem::path getUnaryDirectory() const { return mUnDir; };

private:
    Unaries(const std::filesystem::path& unDir,
        const std::unordered_map<std::size_t, std::filesystem::path> unFiles, cv::Size imgSize,
        double subsample, double sigma, bool removeCamMotion, const cv::Mat& gaussian);
    static cv::Mat getOOPO1(const cv::Mat& img);
    static void writeChunk(const Images& imgContainer, const std::filesystem::path& unDir,
        std::pair<std::size_t, std::size_t> bounds, const std::vector<cv::Mat>& unaries,
        std::size_t start);
    static void writeProperties(
        const std::filesystem::path& unDir, double subsample, double sigma, bool removeCamMotion);
    static std::tuple<double, double, bool> readProperties(const std::filesystem::path& unDir);
    static void writeUnary(const std::filesystem::path& file, const cv::Mat& unary);

    static std::filesystem::path getFileName(
        const std::filesystem::path& unDir, const std::filesystem::path& stem);

private:
    std::filesystem::path mUnDir;
    std::unordered_map<std::size_t, std::filesystem::path> mUnFiles;
    cv::Size mImgSize;
    double mSubsample;
    double mSigma;
    bool mRemoveCamMotion;
    cv::Mat mGaussian;
};
} // namespace ht
#endif // TRACKER_UNARIES_H
