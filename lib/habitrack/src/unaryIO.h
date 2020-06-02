#ifndef HT_UNARY_IO_H
#define HT_UNARY_IO_H

#include <cereal/archives/json.hpp>
#include <cereal/types/unordered_map.hpp>

#include "io/io.h"
#include <opencv2/core.hpp>

namespace cereal
{
template <class Archive>
void serialize(Archive& archive, cv::Point2f& pt)
{
    archive(make_nvp("x", pt.x), make_nvp("y", pt.y));
}
} // namespace cereal
#endif // HT_UNARY_IO_H
