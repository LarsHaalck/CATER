#include <habitrack/image-processing/features.h>
#include <habitrack/image-processing/images.h>
#include <habitrack/image-processing/matches.h>
#include <habitrack/tracker/manualUnaries.h>
#include <habitrack/tracker/tracker.h>
#include <habitrack/tracker/unaries.h>

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>

#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>

// https://stackoverflow.com/a/27030598
template <typename T>
std::vector<double> linspace(T start_in, T end_in, int num_in)
{
    std::vector<double> linspaced;

    double start = static_cast<double>(start_in);
    double end = static_cast<double>(end_in);
    double num = static_cast<double>(num_in);

    if (num == 0)
        return linspaced;
    if (num == 1)
    {
        linspaced.push_back(start);
        return linspaced;
    }

    double delta = (end - start) / (num - 1);
    for (int i = 0; i < num - 1; ++i)
        linspaced.push_back(start + delta * i);

    // I want to ensure that start and end
    // are exactly the same as the input
    linspaced.push_back(end);
    return linspaced;
}

void write_csv(const ht::Detections& dets, double perc)
{
    std::string name = std::string("net_perc_") + std::to_string(perc) + "_fg.csv";
    std::ofstream csv(name, std::ofstream::out);
    for (auto&& d : dets.cdata())
    {
        auto pos = d.second.position;
        csv << pos.x << "," << pos.y << std::endl;
    }
}

using path = std::filesystem::path;
path base_path = "/data/Dropbox/Selected Ontogeny Data/Videos/Ant13/Ant13R1";
path img_folder = base_path / "imgs";
path match_folder = base_path / "imgs_output" / "now" / "matches";
path un_folder = base_path / "imgs_output" / "now" / "unaries";

std::size_t start_frame = 619;
std::size_t end_frame = 13724;

int main()
{
    spdlog::set_level(spdlog::level::info);

    auto imgs = ht::Images(img_folder);
    auto trafos = ht::matches::getTrafos(match_folder, ht::GeometricType::Homography);
    auto uns = ht::Unaries::fromDir(imgs, un_folder, start_frame, end_frame);
    auto manual_uns = ht::ManualUnaries::fromDir(un_folder, 0.8, 9, imgs.getImgSize());
    auto settings = ht::Tracker::Settings {0.8, 15, 6.0, 4.0, false, 5, 3, 0};
    /* auto gt = read_gt(); */

    double perc = 0.0;
    int num_images = end_frame - start_frame;
    while (perc <= 1)
    {
        int num = std::round(perc * num_images);
        if (num == 0)
            num = 2;

        auto ids_d = linspace(0, num_images - 1, num);
        std::vector<int> ids;
        ids.reserve(ids_d.size());
        std::transform(std::begin(ids_d), std::end(ids_d), std::back_inserter(ids),
            [](double e) { return std::round(e); });

        /* for (auto id : ids) */
        /*     manual_uns.insert(id, gt[id]); */

        auto dets = ht::Tracker::track(uns, manual_uns, settings, trafos);
        write_csv(dets, perc);
    }

    return 0;
}
