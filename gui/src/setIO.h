#ifndef HT_MAT_IO_H
#define HT_MAT_IO_H

#include <filesystem>
#include <unordered_set>

namespace gui
{
void saveSet(const std::filesystem::path& setFile, const std::unordered_set<std::size_t>& set);
std::unordered_set<std::size_t> loadSet(const std::filesystem::path& setFile);
} // namespace gui

#endif // HT_MAT_IO_H
