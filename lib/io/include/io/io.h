#ifndef HABITRACK_IO_H
#define HABITRACK_IO_H

#include <filesystem>
#include <fstream>

#include <opencv2/core/persistence.hpp>

namespace ht::io
{
template <typename T>
inline void checkStream(const T& stream, const std::filesystem::path& file)
{
    if (!stream.is_open())
    {
        throw std::filesystem::filesystem_error(
            "Error opening file", file, std::make_error_code(std::errc::io_error));
    }
}

inline void checkFileStorage(const cv::FileStorage& storage, const std::filesystem::path& file)
{
    if (!storage.isOpened())
    {
        throw std::filesystem::filesystem_error(
            "Error opening file", file, std::make_error_code(std::errc::io_error));
    }
}
} // namespace ht

#endif // HABITRACK_IO_H
