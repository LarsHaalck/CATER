#ifndef CATER_MODEL_FMT_H
#define CATER_MODEL_FMT_H

#include <spdlog/fmt/ostr.h>
#include <cater/model/preferences.h>

template <>
struct fmt::formatter<ct::Preferences> : fmt::ostream_formatter
{
};
#endif // CATER_MODEL_FMT_H
