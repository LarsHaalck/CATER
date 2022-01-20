#ifndef HABITRACK_UTIL_LOG
#define HABITRACK_UTIL_LOG

#include <filesystem>

namespace ht
{
void setLogFileTo(const std::filesystem::path& file);
} // namespace ht
#endif // HABITRACK_UTIL_LOG
