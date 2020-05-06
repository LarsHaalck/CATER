#include "habitrack/imageContainer.h"
#include "habitrack/featureContainer.h"
#include "habitrack/matchesContainer.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
using namespace std;

constexpr std::string_view img_folder = "/home/lars/data/ant_long/imgs";
constexpr std::string_view ft_folder = "/home/lars/data/ant_long/fts";
constexpr std::string_view match_folder = "/home/lars/data/ant_long/matches";

int main()
{
    ht::ImageContainer imgs{img_folder, ht::ReadMode::Gray};

    ht::FeatureContainer fts;
    if (ht::FeatureContainer::isComputed(imgs, ft_folder, ht::FeatureType::ORB))
    {
        cout << "Skipping Feature computation..." << endl;
        fts = ht::FeatureContainer::fromDir(
            imgs, "/home/lars/data/ant_long/fts", ht::FeatureType::ORB);
    }
    else
    {
        fts = ht::FeatureContainer::compute(
            imgs, "/home/lars/data/ant_long/fts", ht::FeatureType::ORB, 5000, 200);
    }

    ht::MatchesContainer matches;
    if (ht::MatchesContainer::isComputed(match_folder, ht::GeometricType::Similarity))
    {
        cout << "yes" << endl;
    }
    else
    {
        matches = ht::MatchesContainer::compute(match_folder, ht::GeometricType::Similarity, fts, ht::MatchType::Exhaustive);
    }


    return 0;
}
