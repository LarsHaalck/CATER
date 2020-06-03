#include "habitrack/detections.h"
namespace ht
{

void Detections::insert(std::size_t idx, const Detection& detection)
{
    mDetections.insert({idx, detection});
}

Detection Detections::at(std::size_t idx) const
{
    return mDetections.at(idx);
}

bool Detections::exists(std::size_t idx) const
{
    return mDetections.count(idx);
}

} // namespace ht
