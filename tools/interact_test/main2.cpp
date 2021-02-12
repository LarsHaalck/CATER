#include "habitrack/manualUnaries.h"
#include "habitrack/tracker2.h"
#include "habitrack/unaries.h"
#include "image-processing/features.h"
#include "image-processing/images.h"
#include "image-processing/matches.h"
#include "kde.h"

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
path detectFile = base_path / "imgs_output" / "now" / "detections_gt.yml";

std::size_t start_frame = 619;
std::size_t end_frame = 13724;
std::size_t chunk = 100;

std::vector<double> random_choice_even(
    const std::vector<double>& vec, std::size_t n)
{
    std::vector<double> subset;
    subset.reserve(n);
    float step = static_cast<float>(vec.size()) / n;
    for (std::size_t i = 0; i < n; i++)
    {
        auto idx = static_cast<int>(std::round(i * step));
        subset.push_back(vec[idx]);
    }

    return subset;
}


int main(int argc, char** argv)
{
    spdlog::set_level(spdlog::level::info);

    if (argc != 4)
        return -1;

    std::string a1 {argv[1]};
    std::string a2 {argv[2]};
    std::string a3 {argv[3]};
    spdlog::info("Called {}-{}-{}", a1, a2, a3);

    /* auto imgs = ht::Images(img_folder); */
    /* auto detections = ht::Detections::fromDir(detectFile); */
    /* auto manual_uns = ht::ManualUnaries(0.8, imgs.getImgSize()); */
    /* for (auto detect : detections.cdata()) */
    /*     manual_uns.insert(detect.first, detect.second.position); */
    /* manual_uns.save(un_folder); */

    auto imgs = ht::Images(img_folder);
    auto trafos = ht::matches::getTrafos(match_folder, ht::GeometricType::Homography);
    auto uns = ht::Unaries::fromDir(imgs, un_folder, start_frame, end_frame);
    auto manual_uns_all = ht::ManualUnaries::fromDir(un_folder, 0.8, imgs.getImgSize());
    auto settings = ht::Tracker2::Settings {0.8, 25, 250, 4, false, 5, 3, chunk};
    std::string end = a1 + "-" + a2 + "-" + a3 + "_";

    std::vector<double> frames_all;
    frames_all.reserve(manual_uns_all.size());
    for (const auto& pointPair : manual_uns_all)
        frames_all.push_back(pointPair.first);
    std::sort(std::begin(frames_all), std::end(frames_all));

    std::size_t step = 100;
    std::size_t n = 5251 + step; // so the first loop will implicitly handle mikes number

    while (n > step)
    {
        std::size_t k = n - step;

        auto manual_uns = manual_uns_all;
        auto frames_subset = random_choice_even(frames_all, k);

        spdlog::warn("S {}", frames_subset.size());


        auto manual_uns_subset = ht::ManualUnaries(0.8, imgs.getImgSize());
        for (auto f : frames_subset)
            manual_uns_subset.insert(f, manual_uns.unaryPointAt(f));
        manual_uns = std::move(manual_uns_subset);

        spdlog::info("Running Tracker with {} manual unaries", manual_uns.size());
        auto detections = ht::Tracker2::track(uns, manual_uns, settings, trafos);
        detections.save(
            base_path / ("detections_" + end + std::to_string(manual_uns.size()) + ".yaml"));

        n = k;
    }

    return 0;
}
