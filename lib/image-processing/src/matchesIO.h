#ifndef HT_MATCHES_IO_H
#define HT_MATCHES_IO_H

#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/vector.hpp>
#include <opencv2/core.hpp>

#include "io/io.h"

namespace cereal
{
template <class Archive>
void serialize(Archive& archive, cv::DMatch& match)
{
    archive(match.queryIdx, match.trainIdx, match.distance);
}

template <class Archive, typename T, typename S>
void serialize(Archive& archive, std::pair<T, S>& pair)
{
    archive(pair.first, pair.second);
}
} // namespace cereal

#endif // HT_MATCHES_IO_H
