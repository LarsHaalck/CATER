#ifndef TRACKER_MANUAL_UNARIES_H
#define TRACKER_MANUAL_UNARIES_H

#include <filesystem>
#include <opencv2/core.hpp>
#include <unordered_map>

namespace ct
{
class ManualUnaries
{
private:
    double mSubsample;
    int mUnarySize;
    cv::Size mImgSize;
    std::unordered_map<std::size_t, cv::Point2f> mPoints;

    // stores a single centered manual unary embedded in an image three times the size of
    // the input data (mImgSize) to be able to calculate manual unaries efficiently on the fly
    cv::Mat mBluePrint;

public:
    ManualUnaries();
    ManualUnaries(double subsample, int unarySize, cv::Size imgSize);

    static ManualUnaries fromDir(
        const std::filesystem::path& file, double subsample, int unarySize, cv::Size imgSize);
    void save(const std::filesystem::path& file);

    cv::Mat unaryAt(std::size_t id) const;
    cv::Mat previewUnaryAt(std::size_t id) const;

    cv::Point2f unaryPointAt(std::size_t id) const;
    std::size_t size() const { return mPoints.size(); }

    void insert(std::size_t id, cv::Point2f pt);
    void clear(std::size_t id);
    bool exists(std::size_t id) const;

    auto begin() const -> typename decltype(mPoints)::const_iterator { return mPoints.cbegin(); }
    auto end() const -> typename decltype(mPoints)::const_iterator { return mPoints.cend(); }

private:
    ManualUnaries(double subsample, int unarySize, cv::Size imgSize,
        const std::unordered_map<std::size_t, cv::Point2f>& mPoints);

    cv::Mat getUnaryFromPoint(const cv::Point2f& pt) const;
    cv::Mat getBluePrint();
};
} // namespace ct
#endif // TRACKER_MANUAL_UNARIES_H
