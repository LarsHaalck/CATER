#ifndef HABITRACK_RESULTS_IO_H
#define HABITRACK_RESULTS_IO_H

#include "model/preferences.h"
#include <filesystem>

namespace cv
{
class FileStorage;
} // namespace cv

namespace gui
{
namespace detail
{
    void savePreferences(cv::FileStorage& fs, const model::Preferences& prefs);
    model::Preferences loadPreferences(const cv::FileStorage& fs);
} // namespace detail

void saveResults(const std::filesystem::path& resultFile, const model::Preferences& prefs,
    const std::filesystem::path& imgFolder, std::size_t startFrame, std::size_t endFrame);

std::tuple<model::Preferences, std::filesystem::path, std::size_t, std::size_t> loadResults(
    const std::filesystem::path& resultFile);

} // namespace gui
#endif // HABITRACK_RESULTS_IO_H
