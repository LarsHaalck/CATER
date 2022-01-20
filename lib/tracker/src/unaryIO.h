#ifndef HT_UNARY_IO_H
#define HT_UNARY_IO_H

#include <cereal/archives/json.hpp>
#include <cereal/types/unordered_map.hpp>

#include <habitrack/io/io.h>
#include <opencv2/core.hpp>

namespace cereal
{
template <class Archive, class T>
void serialize(Archive& archive, cv::Point_<T>& pt)
{
    archive(make_nvp("x", pt.x), make_nvp("y", pt.y));
}
} // namespace cereal
#endif // HT_UNARY_IO_H
