#include "image-processing/features.h"
#include "image-processing/images.h"
#include "image-processing/matches.h"
#include "tracker/unaries.h"

/* #include "habitrack/transformation.h" */

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
constexpr ht::GeometricType geom_type = ht::GeometricType::Homography;
constexpr std::size_t cache_size = 2000;

int main()
{
    spdlog::set_level(spdlog::level::debug);
    /* spdlog::cfg::load_env_levels(); */

    ht::Images imgs {img_folder, ht::Images::ReadMode::Gray};

    ht::Features fts;
    if (ht::Features::isComputed(imgs, ft_folder, ht::FeatureType::ORB))
    {
        spdlog::info("Skipping Feature computation");
        fts = ht::Features::fromDir(imgs, ft_folder, ht::FeatureType::ORB);
    }
    else
        fts = ht::Features::compute(imgs, ft_folder, ht::FeatureType::ORB, 2000, cache_size);

    if (ht::matches::isComputed(match_folder, geom_type))
    {
        spdlog::info("Skipping Matches computation");
    }
    else
    {
        ht::matches::compute(match_folder, geom_type, fts, ht::matches::MatchType::Windowed, 2, 0.0,
            nullptr, cache_size);
    }

    auto types
        = ht::matches::getConnectedTypes(match_folder, ht::GeometricType::Homography, imgs.size());
    if (static_cast<unsigned int>(types & ht::GeometricType::Homography))
        spdlog::info("usable for homography");
    else
    {
        spdlog::warn("not usable for homography");
        return 0;
    }

    imgs = ht::Images {img_folder, ht::Images::ReadMode::Unchanged};
    auto trafos = ht::matches::getTrafos(match_folder, geom_type);
    ht::Unaries uns;

    if (ht::Unaries::isComputed(imgs, un_folder))
    {
        spdlog::info("Skipping Unary computation");
        uns = ht::Unaries::fromDir(imgs, un_folder);
    }
    else
        uns = ht::Unaries::compute(imgs, un_folder, 0, -1, true, 0.8, 200.0, trafos, cache_size);

    return 0;
}
