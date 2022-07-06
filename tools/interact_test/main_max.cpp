#include <habitrack/image-processing/features.h>
#include <habitrack/image-processing/images.h>
#include <habitrack/image-processing/matches.h>
#include <habitrack/tracker/manualUnaries.h>
#include <habitrack/tracker/tracker.h>
#include <habitrack/tracker/unaries.h>

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <random>
#include <set>

#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>

using path = std::filesystem::path;
path base_path = "/data";
path img_folder = base_path / "imgs";
path match_folder = base_path / "imgs_output" / "now" / "matches";
path un_folder = base_path / "imgs_output" / "now" / "unaries";

std::size_t start_frame = 619;
std::size_t end_frame = 13724;
std::size_t chunk = 100;

std::random_device rd;
std::mt19937 gen(rd());

int main(int argc, char** argv)
{
    spdlog::set_level(spdlog::level::info);

    if (argc != 4)
        return -1;

    std::string a1 {argv[1]};
    std::string a2 {argv[2]};
    std::string a3 {argv[3]};
    spdlog::info("Called {}-{}-{}", a1, a2, a3);

    auto imgs = ht::Images(img_folder);
    auto trafos = ht::matches::getTrafos(match_folder, ht::GeometricType::Homography);
    auto uns = ht::Unaries::fromDir(imgs, un_folder, start_frame, end_frame);
    auto manual_uns = ht::ManualUnaries::fromDir(un_folder, 0.8, 9, imgs.getImgSize());
    auto settings = ht::Tracker::Settings {0.8, 25, 6, 4, true, 5, 3, chunk};
    /* auto detections = ht::Tracker::track(uns, manual_uns, settings, trafos); */
    std::string end = a1 + "-" + a2 + "-" + a3 + "_";
    /* detections.save(base_path / ("detections_gt.yaml")); */

    std::vector<double> frames;
    frames.reserve(manual_uns.size());
    for (const auto& pointPair : manual_uns)
        frames.push_back(pointPair.first);
    std::sort(std::begin(frames), std::end(frames));


    std::size_t step = 100;
    std::size_t n = 5234 + step; // so the first loop will implicitly handle mikes number

    while (n > 0)
    {
        std::size_t k = 0;
        if (n >= step)
            k = n - step;
        else
            k = 0;

        auto manual_uns_subset = ht::ManualUnaries(0.8, 9, imgs.getImgSize());
        for (auto f : frames)
            manual_uns_subset.insert(f, manual_uns.unaryPointAt(f));
        manual_uns = std::move(manual_uns_subset);

        spdlog::info("Running Tracker with {} manual unaries", manual_uns.size());
        auto detections = ht::Tracker::track(uns, manual_uns, settings, trafos);
        detections.save(
            base_path / ("detections_" + end + std::to_string(manual_uns.size()) + ".yaml"));

        n = k;
    }

    return 0;
}
