#include "habitrack/manualUnaries.h"
#include "habitrack/tracker.h"
#include "habitrack/unaries.h"

#include "image-processing/features.h"
#include "image-processing/images.h"
#include "image-processing/matches.h"

#include "kde.h"

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <random>

#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>

#include <fstream>

using path = std::filesystem::path;
path base_path = "/data/Dropbox/Selected Ontogeny Data/Videos/Ant13/Ant13R1";
path img_folder = base_path / "imgs";
path match_folder = base_path / "imgs_output" / "now" / "matches";
path un_folder = base_path / "imgs_output" / "now" / "unaries";

std::size_t start_frame = 619;
std::size_t end_frame = 13724;
std::size_t chunk = 100;

std::random_device rd;
std::mt19937 gen(rd());

std::vector<double> random_choice(
    const std::vector<double>& vec, std::size_t n, std::vector<double> probs)
{
    std::vector<double> subset;
    subset.reserve(n);
    std::discrete_distribution<std::size_t> dist(std::begin(probs), std::end(probs));
    for (std::size_t i = 0; i < n; i++)
    {
        auto idx = dist(gen);
        subset.push_back(vec[idx]);
        probs[idx] = 0;
        dist = std::discrete_distribution<std::size_t>(std::begin(probs), std::end(probs));
    }

    return subset;
}

int main()
{
    spdlog::set_level(spdlog::level::info);

    auto imgs = ht::Images(img_folder);
    auto trafos = ht::matches::getTrafos(match_folder, ht::GeometricType::Homography);
    auto uns = ht::Unaries::fromDir(imgs, un_folder, start_frame, end_frame);
    auto manual_uns = ht::ManualUnaries::fromDir(un_folder, 0.8, imgs.getImgSize());
    auto settings = ht::Tracker::Settings {0.8, 25, 250, 4, false, 5, 3, chunk};
    auto detections = ht::Tracker::track(uns, manual_uns, settings, trafos);

    std::vector<double> frames;
    frames.reserve(manual_uns.size());
    for (const auto& pointPair : manual_uns)
        frames.push_back(pointPair.first);
    std::sort(std::begin(frames), std::end(frames));

    std::size_t step = 100;

    // put here if it should be done before
    /* auto kde = KDE(KDE::BandwidthType::Silverman); */
    /* kde.fit(frames); */

    while (manual_uns.size() > step)
    {
        std::size_t k = manual_uns.size() - step;
        auto kde = KDE(KDE::BandwidthType::Silverman);
        kde.fit(frames);
        auto frames_prob = kde.pdf(frames);

        std::vector<double> frames_prob_inv;
        frames_prob_inv.resize(frames_prob.size());
        auto max = *std::max_element(std::begin(frames_prob), std::end(frames_prob));
        std::transform(std::begin(frames_prob), std::end(frames_prob), std::begin(frames_prob_inv),
            [max](auto elem) { return max - elem; });

        auto frames_subset0 = random_choice(frames, k / 2, frames_prob);
        auto frames_subset1 = random_choice(frames, k / 2, frames_prob_inv);

        frames_subset0.insert(std::end(frames_subset0),
            std::make_move_iterator(std::begin(frames_subset1)),
            std::make_move_iterator(std::end(frames_subset1)));

        frames = std::move(frames_subset0);
        auto manual_uns_subset = ht::ManualUnaries(0.8, imgs.getImgSize());
        for (auto f : frames)
            manual_uns_subset.insert(f, manual_uns.unaryPointAt(f));
        manual_uns = std::move(manual_uns_subset);

        spdlog::info("Running Tracker with {} manual unaries", manual_uns.size());
        detections = ht::Tracker::track(uns, manual_uns, settings, trafos);
        detections.save(base_path / "detections_" / std::to_string(manual_uns.size()) / ".yaml");
    }

    return 0;
}
