#ifndef CATER_UTIL_LOG
#define CATER_UTIL_LOG

#include <filesystem>

namespace ct
{
void setLogFileTo(const std::filesystem::path& file);
} // namespace ct
#endif // CATER_UTIL_LOG
