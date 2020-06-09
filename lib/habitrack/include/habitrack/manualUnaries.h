#ifndef HABITRACK_MANUAL_UNARIES_H
#define HABITRACK_MANUAL_UNARIES_H

#include <filesystem>
#include <opencv2/core.hpp>
#include <unordered_map>

namespace ht
{
class ManualUnaries
{
public:
    ManualUnaries();
    ManualUnaries(double subsample, cv::Size imgSize);

    static ManualUnaries fromDir(
        const std::filesystem::path& file, double subsample, cv::Size imgSize);
    void save(const std::filesystem::path& file);

    cv::Mat unaryAt(std::size_t id) const;
    cv::Mat previewUnaryAt(std::size_t id) const;
    cv::Point2f unaryPointAt(std::size_t id) const;
    std::size_t size() const { return mUnaries.size(); }

    void insert(std::size_t id, cv::Point2f pt);
    void clear(std::size_t id);
    bool exists(std::size_t id) const;

private:
    ManualUnaries(double subsample, cv::Size imgSize,
        const std::unordered_map<std::size_t, cv::Point2f>& mPoints);

private:
    double mSubsample;
    cv::Size mImgSize;
    std::unordered_map<std::size_t, cv::Point2f> mPoints; // loaded form file
    std::unordered_map<std::size_t, cv::Mat> mUnaries; // generated from points
};
} // namespace ht
#endif // HABITRACK_MANUAL_UNARIES_H
