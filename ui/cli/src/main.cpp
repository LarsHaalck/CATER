#include <filesystem>
#include <iostream>
#include <spdlog/spdlog.h>
#include <string>
#include <vector>

#include <cater/model/model.h>
#include <cater/panorama/panoramaEngine.h>
#include <cater/util/log.h>

#include "CLI11.hpp"

namespace fs = std::filesystem;

// TODO: copied from tui (refactor this)
std::vector<cv::Point> getDetections(const ct::Model& cater)
{
    auto dets = cater.detections();
    auto data = dets.cdata();
    std::vector<cv::Point> vec(dets.size());
    std::transform(std::begin(data), std::end(data), std::begin(vec),
        [](auto pair) { return pair.second.position; });
    // repeat last element to have as many detections as trafos
    vec.push_back(*vec.rbegin());
    return vec;
}

// TODO: copied from tui (refactor this)
void generate_panorama(const ct::PanoramaSettings& settings, const std::vector<std::string>& files)
{
    std::vector<ct::Images> images;
    std::vector<fs::path> data;

    std::vector<cv::Point> pts;
    std::vector<std::size_t> chunkSizes;
    for (auto resFile : files)
    {
        ct::Model cater;
        cater.loadResultsFile(resFile);

        ct::Images currImages = cater.images();
        currImages.clip(cater.getStartFrame(), cater.getEndFrame() + 1);

        std::vector<cv::Point> currPts;
        if (settings.overlayPoints)
            currPts = getDetections(cater);

        auto outPath = cater.getOutputPath() / "panorama";

        ct::setLogFileTo(outPath / "log.txt");
        ct::PanoramaEngine::runSingle(
            currImages, outPath, settings, currPts, cater.getPreferences().chunkSize);

        images.push_back(currImages);
        data.push_back(outPath);
        pts.insert(std::end(pts), std::begin(currPts), std::end(currPts));
        chunkSizes.push_back(cater.getPreferences().chunkSize);
    }

    if (files.size() <= 1)
        return;

    auto outFolder = fs::path(files[0]).parent_path() / "panorama_combined";
    ct::setLogFileTo(outFolder / "log.txt");
    for (auto resFile : files)
        spdlog::info("Adding res file for combined panorama: {}", resFile);

    try
    {
        ct::PanoramaEngine::runMulti(images, data, outFolder, settings, pts, chunkSizes);
    }
    catch (const std::runtime_error& e)
    {
        spdlog::error(e.what());
    }
}

int main(int argc, char* argv[])
{
    spdlog::set_level(spdlog::level::debug);

    CLI::App app {"App description"};

    // general options

    /////////////////////////////////////////////////////////////////
    // add pano(rama) command
    auto* pano = app.add_subcommand("pano", "panorama subcommand");

    // add pano arguments
    int rows = 2000;
    int cols = 2000;
    int num_features = 500;
    bool overlay_centers = false;
    bool overlay_points = false;
    std::vector<std::string> files;

    pano->add_option("--rows", rows, "num rows of panorama")->capture_default_str();
    pano->add_option("--cols", cols, "num cols of panorama")->capture_default_str();
    pano->add_flag("--overlay-centers", overlay_centers, "generate pano with overlayed centers");
    pano->add_flag("--overlay-points", overlay_points, "generate pano with overlayed detections");
    pano->add_option("files", files, "yml result files to process");

    /////////////////////////////////////////////////////////////////
    // add track command
    auto* track = app.add_subcommand("track", "track subcommand");

    // add track arguments
    bool partial_pipeline = false;
    int cache_size = 400;
    int chunk_size = 100;

    track->add_flag("--partial", partial_pipeline, "fun fill pipeline or only optimize unaries");
    track->add_option("--cache", cache_size, "cache size")->capture_default_str();
    track->add_option("--chunk", chunk_size, "chunk size")->capture_default_str();
    track->add_option("files", files, "yml result files to process");

    /////////////////////////////////////////////////////////////////
    // add init command
    auto* init
        = app.add_subcommand("init", "init subcommand to write results.yml from image folder");
    init->add_option("files", files, "yml result files to process");

    // reuiqred at at least 1 and maximum 1 subcommand
    app.require_subcommand(1, 1);

    CLI11_PARSE(app, argc, argv);

    if (files.empty())
    {
        std::cout << "At least one file needs to be supplied" << std::endl;
        return 1;
    }


    // process commands:
    if (app.got_subcommand(pano))
    {
        ct::PanoramaSettings settings;
        settings.rows = rows;
        settings.cols = cols;
        settings.numFeatures = num_features;
        settings.overlayCenters = overlay_centers;
        settings.overlayPoints = overlay_points;

        generate_panorama(settings, files);
    }

    if (app.got_subcommand(track))
    {
        if (files.size() > 1)
        {
            std::cout << "For track only one file is allowed" << std::endl;
            return 1;
        }
        // set prefs
        auto prefs = ct::Preferences();
        prefs.cacheSize = cache_size;
        prefs.chunkSize = chunk_size;

        ct::Model model;
        model.setPreferences(prefs);
        model.loadResultsFile(files[0]);

        if (partial_pipeline)
            model.optimizeUnaries();
        else
            model.runFullPipeline();

        model.saveResultsFile();
    }

    if (app.got_subcommand(init))
    {
        for (const auto& f : files)
        {
            ct::Model model;
            model.loadImageFolder(f);
            model.saveResultsFile();
        }
    }

    return 0;
}
