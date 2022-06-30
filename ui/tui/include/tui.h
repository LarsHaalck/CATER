#ifndef TUI_H
#define TUI_H

#include <filesystem>
#include <string>

#include <habitrack/model/model.h>
#include <habitrack/panorama/panoramaEngine.h>
#include <habitrack/util/log.h>

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

    void openImages(const std::filesystem::path& imgPath) { mModel.loadImageFolder(imgPath); }
    void loadResults(const std::filesystem::path& resFile);
    void setStart(const std::string& args) { mModel.setStartFrame(std::stoul(args)); }
    void setEnd(const std::string& args) { mModel.setEndFrame(std::stoul(args)); }
    void save() { mModel.saveResultsFile(); }
    void saveDetections(const std::string& args) { mModel.saveDetections(args); }
    void extractFeatures() { mModel.extractFeatures(); }
    void extractTrafos() { mModel.extractTrafos(); }
    void extractUnaries() { mModel.extractUnaries(); }
    void optimize() { mModel.optimizeUnaries(); }
    void track() { mModel.runFullPipeline(); }
    void video(const std::filesystem::path& videoFile) { mModel.generateVideo(videoFile); }

    std::vector<cv::Point> getDetections(const ht::Model& habitrack) const;

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
    ht::Model mModel;
    std::vector<std::filesystem::path> mPanoFiles;
    std::unordered_map<std::string, std::filesystem::path> mPanoGPSFiles;
    ht::PanoramaSettings mPanoSettings;
};

} // namespace tui
#endif // TUI_H
