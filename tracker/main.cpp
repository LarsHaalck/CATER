#include "habitrack/images.h"
#include "habitrack/features.h"
#include "habitrack/matches.h"

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
using namespace std;

using path = std::filesystem::path;
path base_path = "/home/lars/data/ant_long";
path img_folder = base_path / "imgs";
path ft_folder = base_path / "fts";
path match_folder = base_path / "matches";
constexpr std::size_t cache_size = 200;

int main()
{
    ht::Images imgs {img_folder, ht::ReadMode::Gray};

    ht::Features fts;
    if (ht::Features::isComputed(imgs, ft_folder, ht::FeatureType::ORB))
    {
        cout << "Skipping Feature computation..." << endl;
        fts = ht::Features::fromDir(imgs, ft_folder, ht::FeatureType::ORB);
    }
    else
    {
        fts = ht::Features::compute(
            imgs, ft_folder, ht::FeatureType::ORB, 500, cache_size);
    }

    ht::MatchesContainer matches;
    if (ht::MatchesContainer::isComputed(match_folder, ht::GeometricType::Similarity))
    {
        cout << "Skipping Feature computation..." << endl;
    }
    else
    {
        matches = ht::MatchesContainer::compute(match_folder, ht::GeometricType::Similarity, fts,
            ht::MatchType::Windowed, 2, 0.0, nullptr, cache_size);
    }

    return 0;
}
