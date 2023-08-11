#ifndef CATER_TRACKER_FMT_H
#define CATER_TRACKER_FMT_H

#include <cater/util/fmt.h>
#include <cater/tracker/tracker.h>
#include <spdlog/fmt/ostr.h>

template <>
struct fmt::formatter<ct::Tracker::Settings> : fmt::ostream_formatter
{
};
#endif // CATER_TRACKER_FMT_H
