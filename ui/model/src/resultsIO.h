#ifndef RESULTS_IO_H
#define RESULTS_IO_H

#include "model/preferences.h"
#include <filesystem>

namespace cv
{
class FileStorage;
} // namespace cv

namespace ht
{
namespace detail
{
    void savePreferences(cv::FileStorage& fs, const Preferences& prefs);
    Preferences loadPreferences(const cv::FileStorage& fs);
} // namespace detail

void saveResults(const std::filesystem::path& resultFile, const Preferences& prefs,
    const std::filesystem::path& imgFolder, std::size_t startFrame, std::size_t endFrame);

std::tuple<Preferences, std::filesystem::path, std::size_t, std::size_t> loadResults(
    const std::filesystem::path& resultFile);

} // namespace ht
#endif // RESULTS_IO_H
