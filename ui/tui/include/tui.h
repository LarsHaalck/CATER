#ifndef TUI_H
#define TUI_H

#include <filesystem>
#include <string>

#include "habitrack/habiTrack.h"
#include "panorama/panoramaEngine.h"
#include "util/log.h"

namespace tui
{
class Tui
{
public:
    Tui(int argc, char** argv);
    int run();

private:
    int parse(const std::string& response);
    void prefs(const std::string& args);

    void openImages(const std::filesystem::path& imgPath) { mHabiTrack.loadImageFolder(imgPath); }
    void loadResults(const std::filesystem::path& resFile);
    void setStart(const std::string& args) { mHabiTrack.setStartFrame(std::stoul(args)); }
    void setEnd(const std::string& args) { mHabiTrack.setEndFrame(std::stoul(args)); }
    void save() { mHabiTrack.saveResultsFile(); }
    void extractFeatures() { mHabiTrack.extractFeatures(); }
    void extractTrafos() { mHabiTrack.extractTrafos(); }
    void extractUnaries() { mHabiTrack.extractUnaries(); }
    void optimize() { mHabiTrack.optimizeUnaries(); }
    void track() { mHabiTrack.runFullPipeline(); }
    void video(const std::filesystem::path& videoFile) { mHabiTrack.generateVideo(videoFile); }

    std::vector<cv::Point> getDetections(const ht::HabiTrack& habitrack) const;

    void addPanorama(const std::string& args);
    void listPanorama();
    void generatePanorama();
    void panoramaPrefs(const std::string& args);

    template <typename OutputIt>
    void extractWords(const std::string& string, OutputIt out)
    {
        auto isSpace = [](char c) { return c == ' '; };

        for (auto begin = std::find_if_not(std::begin(string), std::end(string), isSpace);
             begin != std::end(string); ++out)
        {
            auto end = std::find_if(begin, std::end(string), isSpace);
            *out = std::string(begin, end);
            begin = std::find_if_not(end, std::end(string), isSpace);
        }
    }

private:
    ht::HabiTrack mHabiTrack;
    std::vector<std::filesystem::path> mPanoFiles;
    ht::PanoramaSettings mPanoSettings;
};

} // namespace tui
#endif // TUI_H
