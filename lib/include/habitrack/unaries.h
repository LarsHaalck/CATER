#ifndef HABITRACK_UNARIES_H
#define HABITRACK_UNARIES_H

#include <filesystem>
#include <vector>

#include "habitrack/matches.h"
#include "habitrack/types.h"

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
        double subsample = 0.8, const matches::PairwiseTrafos& trafos = {},
        std::size_t cacheSize = 0);

private:
    static cv::Mat getOOPO1(const cv::Mat& img);
    static void writeChunk(const Images& imgContainer, const std::filesystem::path& unDir,
        std::pair<std::size_t, std::size_t> bounds, const std::vector<cv::Mat>& unaries,
        std::size_t start);
    static void writeUnary(const std::filesystem::path& file, const cv::Mat& unary);
    static double getUnaryQuality(const cv::Mat& unary);

    static std::filesystem::path getFileName(const std::filesystem::path& unDir,
        const std::filesystem::path& stem);

    /* static bool isComputed(const Images& imgContainer, const std::filesystem::path& ftDir, */
    /*     FeatureType type, const size_t_vec& ids = size_t_vec()); */

    /* static Features fromDir(const Images& imgContainer, const std::filesystem::path& ftDir, */
    /*     FeatureType type, const size_t_vec& ids = size_t_vec()); */

    /* std::vector<cv::KeyPoint> featureAt(std::size_t idx) const override; */
    /* cv::Mat descriptorAt(std::size_t idx) const override; */
private:
    std::filesystem::path mUnDir;
    std::unordered_map<std::size_t, std::filesystem::path> mUnFiles;
    std::unordered_map<std::size_t, double> mUnQuality;
};
} // namespace ht
#endif // HABITRACK_UNARIES_H
