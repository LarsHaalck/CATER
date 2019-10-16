#ifndef HABITRACK_IO_H
#define HABITRACK_IO_H

#include <filesystem>
#include <fstream>

namespace ht
{
/* template<typename T> */
/* void write(const std::filesystem::path& file, const T& data) */
/* { */
/*     std::ofstream stream (file.string(), std::ios::out | std::ios::binary); */
/*     if (!stream.is_open()) */
/*     { */
/*         throw std::filesystem::filesystem_error("Error opening file", */
/*                 file, std::make_error_code(std::errc::io_error)); */
/*     } */

/*     { */
/*         cereal::PortableBinaryOutputArchive archive(stream); */
/*         archive(fts); */
/*         stream.close(); */
/*     } */
/* } */
} // namespace ht

#endif // HABITRACK_IO_H

