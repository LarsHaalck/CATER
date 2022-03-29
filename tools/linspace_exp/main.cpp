#include <habitrack/image-processing/features.h>
#include <habitrack/image-processing/images.h>
#include <habitrack/image-processing/matches.h>
#include <habitrack/tracker/manualUnaries.h>
#include <habitrack/tracker/tracker.h>
#include <habitrack/tracker/unaries.h>

#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

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

std::vector<cv::Point2d> read_gt(const std::filesystem::path& gt_file)
{
    std::ifstream stream(gt_file.string());
    if (!stream.is_open())
        throw std::runtime_error("Gt file could not be opened");

    std::vector<cv::Point2d> pts;
    for (std::string line; std::getline(stream, line);)
    {
        auto it = line.find(",");
        auto a = static_cast<double>(std::stod(line.substr(0, it)));
        auto b = static_cast<double>(std::stod(line.substr(it + 1)));
        pts.push_back({a, b});
    }
    return pts;
}

using path = std::filesystem::path;
path base_path = "/data/ant-ml/Ant2ZVF";
path img_folder = base_path / "imgs";
path match_folder = base_path / "imgs_output" / "now" / "matches";
path un_folder = base_path / "imgs_output" / "now" / "unaries";
path dets_file = base_path / "imgs_output" / "now" / "detections.yml";

std::size_t start_frame = 0;
std::size_t end_frame = 3222;

int main()
{
    spdlog::set_level(spdlog::level::info);

    auto imgs = ht::Images(img_folder, ht::Images::ReadMode::Unchanged, cv::Vec3d(), cv::Vec2d(),
        cv::Rect2i(448, 28, 1024, 1024));
    auto trafos = ht::matches::getTrafos(match_folder, ht::GeometricType::Homography);
    auto uns = ht::Unaries::fromDir(imgs, un_folder, start_frame, end_frame);
    auto settings = ht::Tracker::Settings {0.8, 25, 6.0, 4.0, false, 5, 3, 0};
    auto dets = ht::Detections::fromDir(dets_file);

    double perc = 0.15;
    int num_images = end_frame - start_frame;
    while (perc <= 1)
    {
        auto manual_uns = ht::ManualUnaries(0.8, 9, imgs.getImgSize());
        std::cout << "perc: " << perc << std::endl;
        int num = std::round(perc * num_images);
        if (num == 0)
            num = 2;

        auto ids_d = linspace(start_frame, end_frame - 1, num);
        std::vector<int> ids;
        ids.reserve(ids_d.size());
        std::transform(std::begin(ids_d), std::end(ids_d), std::back_inserter(ids),
            [](double e) { return std::round(e); });

        for (auto id : ids)
            manual_uns.insert(id, dets.at(id).position);

        auto dets = ht::Tracker::track(uns, manual_uns, settings, trafos);
        write_csv(dets, perc);
        perc += 0.01;
    }

    return 0;
}
