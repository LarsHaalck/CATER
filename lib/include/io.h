#ifndef HABITRACK_IO_H
#define HABITRACK_IO_H

#include <filesystem>
#include <fstream>

namespace ht
{
template <typename T>
inline void checkStream(const T& stream, const std::filesystem::path& file)
{
    if (!stream.is_open())
    {
        throw std::filesystem::filesystem_error("Error opening file",
                file, std::make_error_code(std::errc::io_error));
    }
}
} // namespace ht

#endif // HABITRACK_IO_H

