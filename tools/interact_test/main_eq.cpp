#include <habitrack/image-processing/features.h>
#include <habitrack/image-processing/images.h>
#include <habitrack/image-processing/matches.h>
#include <habitrack/model/resultsIO.h>
#include <habitrack/progressbar/progressBar.h>
#include <habitrack/tracker/interpTracker.h>
#include <habitrack/tracker/manualUnaries.h>
#include <habitrack/tracker/unaries.h>
#include <habitrack/util/algorithm.h>

#include <cxxopts.hpp>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <random>
#include <set>

#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>

using path = std::filesystem::path;
std::size_t chunk = 100;

enum class TrackerType
{
    Interpolate = 1,
    Chunked = 2,
    Continous = 3,

};

std::string get_suffix(const TrackerType& type)
{
    switch (type)
    {
    case TrackerType::Interpolate:
        return "int";
    case TrackerType::Chunked:
        return "ch" + std::to_string(chunk);
    case TrackerType::Continous:
        return "con";
    default:
        return "";
    };
}

int main(int argc, char** argv)
{
    spdlog::set_level(spdlog::level::info);

    path res_file, out_dir;
    double step_perc = 1;
    auto trackerType_int = 1;
    cxxopts::Options options("Interact-Test [Equidistant]", "");
    options.add_options()("r,res_file", "The results file to process", cxxopts::value(res_file))(
        "s,step", "The step size in percent (default 1)", cxxopts::value(step_perc))("t,tracker",
        "The tracker type to be used, 1: interpolate, 2: chunked, 3: continous (default 1)",
        cxxopts::value(trackerType_int))("o,out",
        "The out dir relative to parent of images (default parent path of images)",
        cxxopts::value(out_dir));

    auto result = options.parse(argc, argv);
    if (result.count("res_file") != 1)
    {
        std::cout << options.help() << std::endl;
        return -1;
    }

    auto trackerType = static_cast<TrackerType>(trackerType_int);
    if (trackerType != TrackerType::Chunked)
        chunk = 0;

    std::cout << "You called: " << std::endl;
    std::cout << "r: " << res_file.string() << std::endl;
    std::cout << "s: " << step_perc << std::endl;
    std::cout << "o: " << out_dir << std::endl;

    auto results_tuple = ht::loadResults(res_file);
    auto img_folder = std::get<1>(results_tuple);
    auto base_path = img_folder.parent_path();

    auto start_frame = std::get<2>(results_tuple);
    auto end_frame = std::get<3>(results_tuple);

    auto match_folder = base_path / "imgs_output" / "now" / "matches";
    auto un_folder = base_path / "imgs_output" / "now" / "unaries";
    auto detectFile = base_path / "imgs_output" / "now" / "detections.yml";

    auto imgs = ht::Images(img_folder);
    auto gt = ht::Detections::fromDir(detectFile);
    auto trafos = ht::matches::getTrafos(match_folder, ht::GeometricType::Homography);
    auto uns = ht::Unaries::fromDir(imgs, un_folder, start_frame, end_frame);
    auto settings = ht::Tracker::Settings {0.8, 25, 6, 4, false, 5, 3, chunk};

    out_dir = base_path / out_dir;
    std::filesystem::create_directories(out_dir);

    // max is 10% of all frames
    auto upper_bound = static_cast<std::size_t>(std::ceil(gt.size() / 10.0));
    std::size_t step = std::round(step_perc / 100 * gt.size());

    auto progress = ht::ProgressBar();
    progress.setTotal(upper_bound / step);
    for (std::size_t i = 0; i < upper_bound; i += step)
    {
        auto manual_uns = ht::ManualUnaries(0.8, 9, imgs.getImgSize());
        auto ids = ht::linspace<std::size_t>(start_frame, end_frame - 1, (i == 0) ? 2 : i);

        for (auto id : ids)
            manual_uns.insert(id, gt.at(id).position);

        spdlog::info("Running Tracker with {} manual unaries", manual_uns.size());
        auto detections = ht::Detections();
        if (trackerType == TrackerType::Interpolate)
            detections = ht::InterpTracker::track(uns, manual_uns, settings, trafos);
        else
            detections = ht::Tracker::track(uns, manual_uns, settings, trafos);

        detections.save(out_dir
            / ("detections_eq_" + get_suffix(trackerType) + "_" + std::to_string(manual_uns.size())
                + ".yaml"));
        progress.inc();
    }
    progress.done();

    return 0;
}
