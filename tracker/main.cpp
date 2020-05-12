#include "habitrack/features.h"
#include "habitrack/images.h"
#include "habitrack/matches.h"
#include "habitrack/unaries.h"

#include "habitrack/transformation.h"

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/cfg/env.h>

using path = std::filesystem::path;
path base_path = "/data/ant/ant_short";
path img_folder = base_path / "imgs";
path ft_folder = base_path / "fts";
path match_folder = base_path / "matches";
path un_folder = base_path / "unaries";
constexpr ht::GeometricType geom_type = ht::GeometricType::Homography;
constexpr std::size_t cache_size = 20;

int main()
{
    /* spdlog::cfg::load_env_levels(); */
    spdlog::set_level(spdlog::level::debug);
    ht::Images imgs {img_folder, ht::Images::ReadMode::Gray};

    ht::Features fts;
    if (ht::Features::isComputed(imgs, ft_folder, ht::FeatureType::ORB))
    {
        spdlog::info("Skipping Feature computation");
        fts = ht::Features::fromDir(imgs, ft_folder, ht::FeatureType::ORB);
    }
    else
    {
        fts = ht::Features::compute(imgs, ft_folder, ht::FeatureType::ORB, 2000, cache_size);
    }

    if (ht::matches::isComputed(match_folder, geom_type))
    {
        spdlog::info("Skipping Matches computation");
    }
    else
    {
        ht::matches::compute(match_folder, geom_type, fts,
            ht::matches::MatchType::Windowed, 2, 0.0, nullptr, cache_size);
    }

    /* auto matches = ht::matches::getMatches(match_folder, geom_type); */

    auto types
        = ht::matches::getConnectedTypes(match_folder, ht::GeometricType::Homography, imgs.size());
    if (static_cast<unsigned int>(types & ht::GeometricType::Homography))
        spdlog::info("usable for homography");

    imgs = ht::Images{img_folder, ht::Images::ReadMode::Unchanged};
    auto trafos = ht::matches::getTrafos(match_folder, geom_type);
    ht::Unaries uns;
    uns = ht::Unaries::compute(imgs, un_folder, 0, -1, true, 0.8, trafos, cache_size);

    /* if (ht::Unaries::isComputed(imgs, ft_folder, ht::FeatureType::ORB)) */
    /* { */
    /*     cout << "Skipping Unary computation..." << endl; */
    /*     /1* fts = ht::Features::fromDir(imgs, ft_folder, ht::FeatureType::ORB); *1/ */
    /* } */
    /* else */
    /* { */
    /*     /1* fts = ht::Features::compute(imgs, ft_folder, ht::FeatureType::ORB, 500, cache_size); *1/ */
    /* } */


    return 0;
}
