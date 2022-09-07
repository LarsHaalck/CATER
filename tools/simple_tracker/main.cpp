#include <cater/image-processing/features.h>
#include <cater/image-processing/images.h>
#include <cater/image-processing/matches.h>
#include <cater/tracker/unaries.h>
#include <cater/tracker/manualUnaries.h>
#include <cater/tracker/tracker.h>


#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>

using path = std::filesystem::path;
path base_path = "/data/ant/ant_short";
path img_folder = base_path / "imgs";
path ft_folder = base_path / "fts";
path match_folder = base_path / "matches";
path un_folder = base_path / "unaries";
constexpr ct::GeometricType geom_type = ct::GeometricType::Homography;
constexpr std::size_t cache_size = 2000;

int main()
{
    spdlog::set_level(spdlog::level::debug);

    ct::Images imgs {img_folder};

    ct::Features fts;
    if (ct::Features::isComputed(imgs, ft_folder, ct::FeatureType::ORB))
    {
        spdlog::info("Skipping Feature computation");
        fts = ct::Features::fromDir(imgs, ft_folder, ct::FeatureType::ORB);
    }
    else
        fts = ct::Features::compute(imgs, ft_folder, ct::FeatureType::ORB, 2000, cache_size);

    if (ct::matches::isComputed(match_folder, geom_type))
    {
        spdlog::info("Skipping Matches computation");
    }
    else
    {
        ct::matches::compute(match_folder, geom_type, fts, ct::matches::MatchType::Windowed, 2, 0.0,
            nullptr, cache_size);
    }

    auto types
        = ct::matches::getConnectedTypes(match_folder, ct::GeometricType::Homography, imgs.size());
    if (static_cast<unsigned int>(types & ct::GeometricType::Homography))
        spdlog::info("usable for homography");
    else
    {
        spdlog::warn("not usable for homography");
        return 0;
    }

    imgs = ct::Images {img_folder, ct::Images::ReadMode::Unchanged};
    auto trafos = ct::matches::getTrafos(match_folder, geom_type);
    ct::Unaries uns;

    if (ct::Unaries::isComputed(imgs, un_folder))
    {
        spdlog::info("Skipping Unary computation");
        uns = ct::Unaries::fromDir(imgs, un_folder);
    }
    else
        uns = ct::Unaries::compute(imgs, un_folder, 0, -1, true, 0.8, 200.0, trafos, cache_size);

    auto settings = ct::Tracker::Settings {0.8, 25, 250, 4, false, 5, 3, 0};
    auto manual = ct::ManualUnaries();
    auto detections = ct::Tracker::track(uns, manual, settings, trafos);
    return 0;
}
