#include <cater/image-processing/features.h>
#include <cater/image-processing/images.h>
#include <cater/image-processing/matches.h>
#include <cater/tracker/manualUnaries.h>
#include <cater/util/algorithm.h>
#include <cater/tracker/tracker.h>
#include <cater/tracker/unaries.h>

#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>

void write_csv(const ct::Detections& dets, double perc)
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

    auto imgs = ct::Images(img_folder, ct::Images::ReadMode::Unchanged, cv::Vec3d(), cv::Vec2d(),
        cv::Rect2i(448, 28, 1024, 1024));
    auto trafos = ct::matches::getTrafos(match_folder, ct::GeometricType::Homography);
    auto uns = ct::Unaries::fromDir(imgs, un_folder, start_frame, end_frame);
    auto settings = ct::Tracker::Settings {0.8, 25, 6.0, 4.0, false, 5, 3, 0};
    auto dets = ct::Detections::fromDir(dets_file);

    double perc = 0.15;
    int num_images = end_frame - start_frame;
    while (perc <= 1)
    {
        auto manual_uns = ct::ManualUnaries(0.8, 9, imgs.getImgSize());
        std::cout << "perc: " << perc << std::endl;
        int num = std::round(perc * num_images);
        if (num == 0)
            num = 2;

        auto ids_d = ct::linspace(start_frame, end_frame - 1, num);
        std::vector<int> ids;
        ids.reserve(ids_d.size());
        std::transform(std::begin(ids_d), std::end(ids_d), std::back_inserter(ids),
            [](double e) { return std::round(e); });

        for (auto id : ids)
            manual_uns.insert(id, dets.at(id).position);

        auto dets = ct::Tracker::track(uns, manual_uns, settings, trafos);
        write_csv(dets, perc);
        perc += 0.01;
    }

    return 0;
}
