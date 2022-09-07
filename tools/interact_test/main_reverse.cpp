#include "kde.h"

#include <cater/image-processing/features.h>
#include <cater/image-processing/images.h>
#include <cater/image-processing/matches.h>
#include <cater/tracker/manualUnaries.h>
#include <cater/tracker/tracker.h>
#include <cater/tracker/unaries.h>

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

    auto imgs = ct::Images(img_folder);
    auto trafos = ct::matches::getTrafos(match_folder, ct::GeometricType::Homography);
    auto uns = ct::Unaries::fromDir(imgs, un_folder, start_frame, end_frame);
    auto manual_uns = ct::ManualUnaries::fromDir(un_folder, 0.8, 9, imgs.getImgSize());
    auto settings = ct::Tracker::Settings {0.8, 25, 6, 4, true, 5, 3, chunk};
    /* auto detections = ct::Tracker::track(uns, manual_uns, settings, trafos); */
    std::string end = a1 + "-" + a2 + "-" + a3 + "_";
    /* detections.save(base_path / ("detections_gt.yaml")); */

    std::vector<double> frames;
    frames.reserve(manual_uns.size());
    for (const auto& pointPair : manual_uns)
        frames.push_back(pointPair.first);
    std::sort(std::begin(frames), std::end(frames));

    auto kde = KDE(KDE::BandwidthType::Silverman);
    if (a1 == "b")
        kde = KDE(KDE::BandwidthType::Bisection);
    if (a2 == "b")
        kde.fit(frames);

    std::size_t step = 131;
    /* std::size_t n = 5234 + step; // so the first loop will implicitly handle mikes number */
    std::size_t n = 4912 + step;

    while (n > 2)
    {
        std::size_t k = 0;
        if (n >= step + 2)
            k = n - step;
        else
            k = 2;

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

        std::size_t sum = 0;
        while (sum != k)
        {
            if (a3 == "h")
            {
                auto [subset, ids] = random_choice(frames, k / 2, frames_prob);
                frames_subset0 = subset;
                auto frames_prob_inv_changed = frames_prob_inv;
                for (auto id : ids)
                    frames_prob_inv_changed[id] = 0.0;
                frames_subset1 = std::get<0>(random_choice(frames, k / 2, frames_prob_inv_changed));
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

            auto uniqs
                = std::set<std::size_t>(std::begin(frames_subset0), std::end(frames_subset0));
            spdlog::warn("Uniq {}", uniqs.size());
            sum = uniqs.size();
        }

        frames = std::move(frames_subset0);

        auto manual_uns_subset = ct::ManualUnaries(0.8, 9, imgs.getImgSize());
        for (auto f : frames)
            manual_uns_subset.insert(f, manual_uns.unaryPointAt(f));
        manual_uns = std::move(manual_uns_subset);

        spdlog::info("Running Tracker with {} manual unaries", manual_uns.size());
        auto detections = ct::Tracker::track(uns, manual_uns, settings, trafos);
        detections.save(
            base_path / ("detections_" + end + std::to_string(manual_uns.size()) + ".yaml"));

        n = k;
    }

    return 0;
}
