#include <habitrack/image-processing/features.h>
#include <habitrack/image-processing/images.h>
#include <habitrack/image-processing/matches.h>
#include <habitrack/image-processing/util.h>
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

std::string get_filename(const TrackerType& type, double ratio, const path& folder)
{
    std::stringstream filename;
    filename << "detections_max_" << folder.string() << "_";
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

// sort by max deviation from gt
std::vector<int> get_sorted_ids(
    const ht::Detections& detections, const ht::Detections& gt, std::size_t start_frame)
{
    // sort by max deviation from gt
    auto ids = std::vector<int>(detections.size());
    std::iota(std::begin(ids), std::end(ids), start_frame);
    std::sort(std::begin(ids), std::end(ids),
        [detections, gt](auto a, auto b)
        {
            auto dist1 = ht::util::euclidianDist(detections.at(a).position, gt.at(a).position);
            auto dist2 = ht::util::euclidianDist(detections.at(b).position, gt.at(b).position);
            return dist1 > dist2;
        });

    return ids;
}

int main(int argc, char** argv)
{
    spdlog::set_level(spdlog::level::info);

    path res_file, out_dir;
    double step_perc = 1;
    auto trackerType_int = 1;
    bool incremental = false;

    cxxopts::Options options("Interact-Test [Equidistant]", "");
    options.add_options()("r,res_file", "The results file to process", cxxopts::value(res_file))(
        "s,step", "The step size in percent (default 1)", cxxopts::value(step_perc))("t,tracker",
        "The tracker type to be used, 1: interpolate, 2: chunked, 3: continous (default 1)",
        cxxopts::value(trackerType_int))("o,out",
        "The out dir relative to parent of images (default parent path of images)",
        cxxopts::value(out_dir))("i,incremental",
        "Whether to optimize after every max or in groups (default false)",
        cxxopts::value(incremental));

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
    step_perc /= 100;
    std::size_t steps = std::round(upper_bound / (step_perc * gt.size()));

    auto progress = ht::ProgressBar();
    progress.setTotal(steps);
    auto manual_uns = ht::ManualUnaries(0.8, 9, imgs.getImgSize());
    auto detections = ht::Detections();
    for (std::size_t i = 0; i <= steps; i++)
    {
        auto num_unaries = std::round(i * step_perc * gt.size()) - manual_uns.size();
        std::vector<int> ids;

        // sort by highest, add max to manual unary, run tracker and repeat until step reached
        if (incremental)
        {
            auto manual_uns_copy = manual_uns;
            auto detections_copy = detections;
            ids.reserve(num_unaries);
            for (int k = 0; k < static_cast<int>(num_unaries); k++)
            {
                // sort by max deviation from gt
                auto max_ids = get_sorted_ids(detections_copy, gt, start_frame);
                manual_uns_copy.insert(max_ids[0], gt.at(max_ids[0]).position);
                ids.push_back(max_ids[0]);

                if (trackerType == TrackerType::Interpolate)
                    detections_copy
                        = ht::InterpTracker::track(uns, manual_uns_copy, settings, trafos);
                else
                    detections_copy = ht::Tracker::track(uns, manual_uns_copy, settings, trafos);
            }
        }
        else // sort by highest, and find step many points heuristically without running inbetween
        {
            auto max_ids = get_sorted_ids(detections, gt, start_frame);

            int curr = 0;
            int min_dist = 50;
            while (
                curr < static_cast<int>(max_ids.size()) && ids.size() < num_unaries && min_dist > 0)
            {
                spdlog::debug("Min dist: {}", min_dist);
                // find next element that is atleast 50 from previous elements
                auto new_id = max_ids[curr];
                bool ok = true;
                for (const auto& s : ids)
                {
                    if (std::abs(new_id - s) < min_dist)
                    {
                        spdlog::debug("{} too close to other id {}", new_id, s);
                        ok = false;
                        break;
                    }
                }
                for (const auto& s : manual_uns)
                {
                    if (std::abs(new_id - static_cast<int>(s.first)) < min_dist)
                    {
                        spdlog::debug("{} too close to manual unary {}", new_id, s.first);
                        ok = false;
                        break;
                    }
                }

                if (!ok)
                    curr++;
                else
                    ids.push_back(new_id);

                if (curr == static_cast<int>(max_ids.size() - 1))
                {
                    curr = 0;
                    min_dist -= 10;
                }
            }
        }

        for (auto id : ids)
            manual_uns.insert(id, gt.at(id).position);

        spdlog::info("Running Tracker with {} manual unaries", manual_uns.size());
        if (trackerType == TrackerType::Interpolate)
            detections = ht::InterpTracker::track(uns, manual_uns, settings, trafos);
        else
            detections = ht::Tracker::track(uns, manual_uns, settings, trafos);

        auto ratio = static_cast<double>(manual_uns.size()) / gt.size();
        detections.save(out_dir / get_filename(trackerType, ratio, base_path.stem()));

        progress.inc();
    }
    progress.done();

    return 0;
}
