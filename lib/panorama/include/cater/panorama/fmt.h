#ifndef CATER_PANORAMA_FMT_H
#define CATER_PANORAMA_FMT_H

#include <cater/panorama/panoramaEngine.h>
#include <cater/util/fmt.h>
#include <fmt/ostream.h>

template <>
struct fmt::formatter<ct::PanoramaStage> : fmt::ostream_formatter
{
};
template <>
struct fmt::formatter<ct::PanoramaSettings> : fmt::ostream_formatter
{
};
template <>
struct fmt::formatter<ct::GPSSettings> : fmt::ostream_formatter
{
};
#endif // CATER_PANORAMA_FMT_H
