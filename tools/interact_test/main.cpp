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
#include <set>

#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>

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

std::pair<std::vector<double>, std::vector<std::size_t>> random_choice(
    const std::vector<double>& vec, std::size_t n, std::vector<double> probs)
{
    std::vector<double> subset;
    std::vector<size_t> subset_id;
    subset.reserve(n);
    subset_id.reserve(n);
    std::discrete_distribution<std::size_t> dist(std::begin(probs), std::end(probs));
    for (std::size_t i = 0; i < n; i++)
    {
        auto idx = dist(gen);
        subset_id.push_back(idx);
        subset.push_back(vec[idx]);
        probs[idx] = 0;
        dist = std::discrete_distribution<std::size_t>(std::begin(probs), std::end(probs));
    }

    return {subset, subset_id};
}

std::vector<double> inv_probs(const std::vector<double>& probs)
{
    std::vector<double> probs_inv;
    probs_inv.resize(probs.size());
    auto max = *std::max_element(std::begin(probs), std::end(probs));
    std::transform(std::begin(probs), std::end(probs), std::begin(probs_inv),
        [max](auto elem) { return max - elem; });
    return probs_inv;
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

    auto imgs = ht::Images(img_folder);
    auto trafos = ht::matches::getTrafos(match_folder, ht::GeometricType::Homography);
    auto uns = ht::Unaries::fromDir(imgs, un_folder, start_frame, end_frame);
    auto manual_uns = ht::ManualUnaries::fromDir(un_folder, 0.8, imgs.getImgSize());
    auto settings = ht::Tracker::Settings {0.8, 25, 250, 4, false, 5, 3, chunk};
    /* auto detections = ht::Tracker::track(uns, manual_uns, settings, trafos); */
    std::string end = a1 + "-" + a2 + "-" + a3 + "_";
    /* detections.save( */
    /*     base_path / ("detections_" + end + std::to_string(manual_uns.size()) + ".yaml")); */

    std::vector<double> frames;
    frames.reserve(manual_uns.size());
    for (const auto& pointPair : manual_uns)
        frames.push_back(pointPair.first);
    std::sort(std::begin(frames), std::end(frames));

    std::size_t step = 100;

    auto kde = KDE(KDE::BandwidthType::Silverman);
    if (a1 == "b")
        kde = KDE(KDE::BandwidthType::Bisection);
    if (a2 == "b")
        kde.fit(frames);

    while (manual_uns.size() > step)
    {
        std::size_t k = manual_uns.size() - step;
        if (a2 == "a")
        {
            if (a1 == "s")
                kde = KDE(KDE::BandwidthType::Silverman);
            else
                kde = KDE(KDE::BandwidthType::Bisection);
            kde.fit(frames);
        }
        auto frames_prob = kde.pdf(frames);
        auto frames_prob_inv = inv_probs(frames_prob);

        auto frames_subset0 = std::vector<double>();
        auto frames_subset1 = std::vector<double>();

        if (a3 == "h")
        {
            auto [subset, ids] = random_choice(frames, k / 2, frames_prob);
            frames_subset0 = subset;
            for (auto id : ids)
                frames_prob_inv[id] = 0.0;
            frames_subset1 = std::get<0>(random_choice(frames, k / 2, frames_prob_inv));
        }
        if (a3 == "d")
            frames_subset0 = std::get<0>(random_choice(frames, k, frames_prob));
        if (a3 == "i")
            frames_subset1 = std::get<0>(random_choice(frames, k, frames_prob_inv));

        spdlog::warn("S0 {}, S1 {}, S0 + S1 {}", frames_subset0.size(), frames_subset1.size(),
            frames_subset0.size() + frames_subset1.size());

        frames_subset0.insert(std::end(frames_subset0),
            std::make_move_iterator(std::begin(frames_subset1)),
            std::make_move_iterator(std::end(frames_subset1)));

        frames = std::move(frames_subset0);

        auto uniqs = std::set<std::size_t>(std::begin(frames), std::end(frames));
        spdlog::warn("Uniq {}", uniqs.size());

        auto manual_uns_subset = ht::ManualUnaries(0.8, imgs.getImgSize());
        for (auto f : frames)
            manual_uns_subset.insert(f, manual_uns.unaryPointAt(f));
        manual_uns = std::move(manual_uns_subset);

        spdlog::info("Running Tracker with {} manual unaries", manual_uns.size());
        auto detections = ht::Tracker::track(uns, manual_uns, settings, trafos);
        detections.save(
            base_path / ("detections_" + end + std::to_string(manual_uns.size()) + ".yaml"));
    }

    return 0;
}
