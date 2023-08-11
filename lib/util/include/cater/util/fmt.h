#ifndef CATER_UTIL_FMT_H
#define CATER_UTIL_FMT_H

#include <filesystem>
#include <spdlog/fmt/ostr.h>
#include <opencv2/core.hpp>

template <>
struct fmt::formatter<std::filesystem::path> : fmt::ostream_formatter
{
};

template <>
struct fmt::formatter<cv::Mat> : fmt::ostream_formatter
{
};
#endif // CATER_UTIL_FMT_H
