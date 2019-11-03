#ifndef HABITACK_KEY_FRAME_SELECTOR_H
#define HABITACK_KEY_FRAME_SELECTOR_H

#include <algorithm>
#include <filesystem>
#include <memory>
#include <opencv2/features2d.hpp>
#include <vector>

#include "habitrack/baseFeatureContainer.h"
#include "habitrack/computeBehavior.h"
#include "habitrack/matchesContainer.h"

namespace ht
{
class KeyFrameSelector
{
public:
    KeyFrameSelector(std::shared_ptr<BaseFeatureContainer> ftContainer, GeometricType type,
        const std::filesystem::path& file);
    std::vector<std::size_t> compute(float low, float high, ComputeBehavior behavior);

private:
    std::pair<float, float> getRealLowHigh(float low, float high) const;
    std::pair<float, std::size_t> getMedianDistanceShift(std::size_t idI, std::size_t idJ);

    std::size_t filterViews(
        const std::vector<std::pair<float, std::size_t>>& distOverlapVec, float low, float high);
    bool compareMaxOverlap(const std::pair<float, std::size_t>& lhs,
        const std::pair<float, std::size_t>& rhs, float low, float high) const;
    double calcReprojError(const std::vector<cv::Point2f>& ptsSrc, std::vector<cv::Point2f>& ptsDst,
        const cv::Mat& trafo) const;

    static float l2Dist(const cv::KeyPoint& pt0, const cv::KeyPoint& pt1)
    {
        return (pt0.pt.x - pt1.pt.x) * (pt0.pt.x - pt1.pt.x)
            + (pt0.pt.y - pt1.pt.y) * (pt0.pt.y - pt1.pt.y);
    }

    static float l2Dist(const cv::Point2f& pt0, const cv::Point2f& pt1)
    {
        return (pt0.x - pt1.x) * (pt0.x - pt1.x) + (pt0.y - pt1.y) * (pt0.y - pt1.y);
    }

    // pass by copy because of sorting
    template <typename T>
    T getMedian(std::vector<T> vec) const
    {
        using sizeT = typename std::vector<T>::size_type;
        sizeT size = vec.size();
        if (size == 0)
            return 0;
        std::sort(std::begin(vec), std::end(vec));

        if (size % 2 == 0)
            return (vec[size / 2 - 1] + vec[size / 2]) / 2;
        else
            return vec[size / 2];
    }

    void writeToFile(const std::vector<std::size_t>& keyFrames);
    std::vector<std::size_t> loadFromFile();

private:
    std::shared_ptr<BaseFeatureContainer> mFtContainer;
    GeometricType mType;
    std::filesystem::path mFile;
    std::unique_ptr<MatchesContainer> mMatchesContainer;
    cv::Size mImgSize;
    int mArea;
    bool mIsComputed;

    PairwiseMatches mMatches;
};
} // namespace ht
#endif // HABITACK_KEY_FRAME_SELECTOR_H
