#include <cater/image-processing/features.h>
#include <cater/image-processing/images.h>
#include <cater/image-processing/matches.h>
#include <cater/model/resultsIO.h>
#include <cater/progressbar/progressBar.h>
#include <cater/tracker/interpTracker.h>
#include <cater/tracker/manualUnaries.h>
#include <cater/tracker/unaries.h>
#include <cater/util/algorithm.h>

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

std::string get_filename(const TrackerType& type, double ratio, const path& folder)
{
    std::stringstream filename;
    filename << "detections_eq_" << folder.string() << "_";
    switch (type)
    {
    case TrackerType::Interpolate:
        filename << "int";
        break;
    case TrackerType::Chunked:
        filename << "ch-" + std::to_string(chunk);
        break;
    case TrackerType::Continous:
        filename << "con";
        break;
    default:
        break;
    };

    filename << "_" << std::fixed << std::setprecision(2) << ratio << ".yaml";
    return filename.str();
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

    auto results_tuple = ct::loadResults(res_file);
    auto img_folder = std::get<1>(results_tuple);
    auto base_path = img_folder.parent_path();

    auto start_frame = std::get<2>(results_tuple);
    auto end_frame = std::get<3>(results_tuple);

    auto match_folder = base_path / "imgs_output" / "now" / "matches";
    auto un_folder = base_path / "imgs_output" / "now" / "unaries";
    auto detectFile = base_path / "imgs_output" / "now" / "detections.yml";

    auto imgs = ct::Images(img_folder);
    auto gt = ct::Detections::fromDir(detectFile);
    auto trafos = ct::matches::getTrafos(match_folder, ct::GeometricType::Homography);
    auto uns = ct::Unaries::fromDir(imgs, un_folder, start_frame, end_frame);
    auto settings = ct::Tracker::Settings {0.8, 25, 6, 4, false, 5, 3, chunk};

    out_dir = base_path / out_dir;
    std::filesystem::create_directories(out_dir);

    // max is 10% of all frames
    auto upper_bound = static_cast<std::size_t>(std::ceil(gt.size() / 10.0));
    step_perc /= 100;
    std::size_t steps = std::round(upper_bound / (step_perc * gt.size()));

    auto progress = ct::ProgressBar();
    progress.setTotal(steps);
    for (std::size_t i = 0; i <= steps; i++)
    {
        auto manual_uns = ct::ManualUnaries(0.8, 9, imgs.getImgSize());
        auto num_unaries = std::round(i * step_perc * gt.size());
        if (num_unaries == 0)
            num_unaries = 2;

        auto ids = ct::linspace<std::size_t>(start_frame, end_frame - 1, num_unaries);
        for (auto id : ids)
            manual_uns.insert(id, gt.at(id).position);

        spdlog::info("Running Tracker with {} manual unaries", manual_uns.size());
        auto detections = ct::Detections();
        if (trackerType == TrackerType::Interpolate)
            detections = ct::InterpTracker::track(uns, manual_uns, settings, trafos);
        else
            detections = ct::Tracker::track(uns, manual_uns, settings, trafos);

        auto ratio = static_cast<double>(manual_uns.size()) / gt.size();
        detections.save(out_dir / get_filename(trackerType, ratio, base_path.stem()));
        progress.inc();
    }
    progress.done();

    return 0;
}
