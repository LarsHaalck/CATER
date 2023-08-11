#ifndef CATER_IMG_FMT_H
#define CATER_IMG_FMT_H

#include <cater/image-processing/baseImageContainer.h>
#include <cater/image-processing/geometricType.h>
#include <cater/util/fmt.h>
#include <spdlog/fmt/ostr.h>

template <>
struct fmt::formatter<ct::FeatureType> : fmt::ostream_formatter
{
};

template <>
struct fmt::formatter<ct::GeometricType> : fmt::ostream_formatter
{
};
#endif // CATER_IMG_FMT_H
