#ifndef HABITRACK_RESULTS_IO_H
#define HABITRACK_RESULTS_IO_H

#include "gui/preferences.h"
#include <filesystem>

namespace cv
{
    class FileStorage;
} // namespace cv

namespace gui
{
namespace detail
{
void savePreferences(cv::FileStorage& fs, const Preferences& prefs);
Preferences loadPreferences(const cv::FileStorage& fs);
} // namespace detail
void saveResults(const std::filesystem::path& resultFile, const Preferences& prefs);
} // namespace gui
#endif // HABITRACK_RESULTS_IO_H
