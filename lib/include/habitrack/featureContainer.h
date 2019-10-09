#ifndef HABITRACK_FEATURE_CONTAINER_H
#define HABITRACK_FEATURE_CONTAINER_H

#include <filesystem>
#include <vector>

#include <opencv2/features2d.hpp>

#include "habitrack/preferences.h"

namespace ht
{
    class ImageContainer;
}

namespace cereal
{
template<class Archive>
void save(Archive& archive, const cv::KeyPoint& kp)
{
    archive(kp.pt.x, kp.pt.y, kp.size, kp.angle, kp.response, kp.octave, kp.class_id);
}
} // namespace cereal

namespace ht
{
enum class FeatureType
{
    ORB,
    SIFT
};

class FeatureContainer
{
public:
    FeatureContainer(std::shared_ptr<ImageContainer> imgContainer,
        const std::filesystem::path& ftDir, FeatureType type, std::size_t numFeatures);

    void compute(bool overwrite, std::size_t cacheSize);
    std::vector<cv::KeyPoint> at(std::size_t idx);
private:
    FeatureType getTypeFromFile(const std::filesystem::path& file);
    std::filesystem::path getFtFileName(std::size_t idx);
    cv::Ptr<cv::Feature2D> getFtPtr();
    void writeChunk(std::pair<std::size_t, std::size_t> bounds,
        const std::vector<std::vector<cv::KeyPoint>>& fts,
        const std::vector<cv::Mat>& descs);
private:
    std::shared_ptr<ImageContainer> mImgContainer;
    std::filesystem::path mFtDir;
    FeatureType mType;
    std::size_t mNumFeatures;

    bool mIsComputed;

};
} // namespace ht

#endif // HABITRACK_FEATURE_CONTAINER_H


