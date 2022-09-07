#ifndef CT_FEATURE_IO_H
#define CT_FEATURE_IO_H

#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/vector.hpp>

#include <cater/io/io.h>
#include <opencv2/features2d.hpp>

namespace cereal
{
template <class Archive>
void serialize(Archive& archive, cv::KeyPoint& kp)
{
    archive(kp.pt.x, kp.pt.y, kp.size, kp.angle, kp.response, kp.octave, kp.class_id);
}
} // namespace cereal

#endif // CT_FEATURE_IO_H
