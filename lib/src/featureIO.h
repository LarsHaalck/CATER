#ifndef HT_FEATURE_IO_H
#define HT_FEATURE_IO_H

#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/vector.hpp>
#include <opencv2/features2d.hpp>

#include "io.h"

namespace cereal
{
template <class Archive>
void serialize(Archive& archive, cv::KeyPoint& kp)
{
    archive(kp.pt.x, kp.pt.y, kp.size, kp.angle, kp.response, kp.octave, kp.class_id);
}
} // namespace cereal

#endif // HT_FEATURE_IO_H
