#include <iostream>
#include <memory>
#include <numeric>

#include "habitrack/featureAggregator.h"
#include "habitrack/featureContainer.h"
#include "habitrack/idTranslator.h"
#include "habitrack/imageAggregator.h"
#include "habitrack/imageContainer.h"
#include "habitrack/keyFrameSelector.h"
#include "habitrack/matchesContainer.h"
#include "habitrack/mildRecommender.h"
#include "habitrack/panoramaStitcher.h"
#include "habitrack/keyFrameRecommender.h"
#include "habitrack/mst.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace ht;
void drawImg(const cv::Mat& img)
{
    int key = 0;
    do
    {
        cv::imshow("img", img);
        key = cv::waitKey(0);
    } while (key != 27);
}

std::unordered_map<std::pair<std::size_t, std::size_t>, std::pair<std::size_t, std::size_t>>
getMostProminantTransition(const PairwiseMatches& matches, const std::vector<std::size_t>& sizes)
{
    auto keys = MatchesContainer::getKeyList(matches);
    ht::Translator translator(sizes);

    std::unordered_map<std::pair<std::size_t, std::size_t>,
        std::tuple<std::size_t, std::size_t, std::size_t>>
        transitions;
    for (auto& pair : keys)
    {
        auto [idI, idJ] = pair;
        auto localIdI = translator.globalToLocal(idI);
        auto localIdJ = translator.globalToLocal(idJ);

        auto blockI = localIdI.first;
        auto blockJ = localIdJ.first;
        auto blockPair = std::make_pair(blockI, blockJ);

        if (!transitions.count(blockPair))
            transitions[blockPair] = std::make_tuple(idI, idJ, matches.at(pair).size());
        else
        {
            auto [oldI, oldJ, count] = transitions[blockPair];
            if (matches.at(pair).size() > count)
                transitions[blockPair] = std::make_tuple(idI, idJ, matches.at(pair).size());
        }
    }

    ht::Graph graph(sizes.size(), transitions.size());
    for (auto elem : transitions)
    {
        auto [blockI, blockJ] = elem.first;
        auto [idI, idJ, count] = elem.second;
        std::cout << "Transition from block: " << blockI << " --> " << blockJ << std::endl;
        std::cout << "With frames: " << idI << " --> " << idJ << ", count = " << count << std::endl;
        std::cout << "--------------------------------------------------" << std::endl;

        graph.addEdge(blockI, blockJ, -count);
    }

    auto edges = graph.kruskalMST();

    using Pair = std::pair<std::size_t, std::size_t>;
    std::unordered_map<Pair, Pair> optimalTransitions;

    for (auto blockPair : edges)
    {
        auto [idI, idJ, _] = transitions[blockPair];
        optimalTransitions.insert(std::make_pair(
            std::make_pair(blockPair.first, blockPair.second), std::make_pair(idI, idJ)));
    }

    /* for (auto elem : optimalTransitions) */
    /* { */
    /*     auto [blockI, blockJ] = elem.first; */
    /*     auto [idI, idJ] = elem.second; */
    /*     std::cout << "Mapping " << blockI << " --> " << blockJ << " with " << idI << " --> " << idJ */
    /*               << std::endl; */
    /* } */
    return optimalTransitions;
}

int main()
{
    std::filesystem::path basePath = "/home/lars/data/ontogenyTest";
    std::vector<std::filesystem::path> videoPaths
        = {basePath / "vid3", basePath / "vid4"};

    std::vector<std::shared_ptr<ImageContainer>> imgContainers;
    std::vector<std::shared_ptr<FeatureContainer>> ftSiftContainers;
    std::vector<std::shared_ptr<FeatureContainer>> ftOrbContainers;
    std::vector<std::vector<std::size_t>> keyFrameList;
    std::vector<std::size_t> sizes;


    PairwiseMatches intraMatches;
    PairwiseTrafos intraTrafos;
    for (const auto& path : videoPaths)
    {
        auto imgContainer = std::make_shared<ImageContainer>(path / "imgs");

        // add sift container and keyframes
        auto ftSiftContainer = std::make_shared<FeatureContainer>(
            imgContainer, path / "fts", FeatureType::SIFT, 5000);

        KeyFrameSelector selector(ftSiftContainer, GeometricType::Similarity, path /
            "key_frames.yml");
        auto currKeyFrames = selector.compute(0.3, 0.5, ComputeBehavior::Keep);
        keyFrameList.push_back(currKeyFrames);

        auto matchIntraContainer = std::make_shared<MatchesContainer>(ftSiftContainer,
            path / "kfs/matches_intra", MatchType::Manual, 0,
            GeometricType::Homography | GeometricType::Affinity | GeometricType::Similarity
                | GeometricType::Isometry);
        auto matches = matchIntraContainer->getMatches(GeometricType::Similarity);
        auto trafos = matchIntraContainer->getTrafos(GeometricType::Similarity);


        ftSiftContainers.push_back(std::move(ftSiftContainer));

        // add and compute orb containers for mild recomender
        auto ftOrbContainer = std::make_shared<FeatureContainer>(
            imgContainer, path / "kfs/fts", FeatureType::ORB, 3000);
        ftOrbContainer->compute(1000, ComputeBehavior::Keep, currKeyFrames);
        ftOrbContainers.push_back(std::move(ftOrbContainer));

        // determine size for mild
        auto currSize = imgContainer->getNumImgs();

        // add size
        sizes.push_back(currSize);
        imgContainers.push_back(std::move(imgContainer));
    }

    // aggregate img and feature container
    auto globalImgContainer = std::make_shared<ImageAggregator>(imgContainers);
    auto globalFtSiftContainer = std::make_shared<FeatureAggregator>(ftSiftContainers);

    // match inter video
    auto recommender = std::make_unique<MildRecommender>(ftOrbContainers);
    auto matchContainer = std::make_shared<MatchesContainer>(globalFtSiftContainer,
        basePath / "ivlc/matches", MatchType::Strategy, 1, // TODO: incorporate into ML estimator
        GeometricType::Homography | GeometricType::Affinity | GeometricType::Similarity |
        GeometricType::Isometry, std::move(recommender));

    // list of local keyframes to global keyframes
    auto globalKeyFrames = translate::localToGlobal(keyFrameList, sizes);
    matchContainer->compute(5000, ComputeBehavior::Keep, globalKeyFrames);

    // combined local dense matches and inter video sparse matches
    auto interMatches = matchContainer->getMatches(GeometricType::Similarity);
    auto optimalTransitions = getMostProminantTransition(interMatches, sizes);
    std::vector<std::vector<cv::Mat>> localOptimalTrafos;
    for (auto path : videoPaths)
        localOptimalTrafos.push_back(PanoramaStitcher::loadTrafos(path / "kfs/opt_trafos.bin"));

    // do pano stitching for every wanted type
    auto stitcher = std::make_unique<PanoramaStitcher>(globalImgContainer, globalFtSiftContainer,
        matches, trafos, globalKeyFrames, GeometricType::Similarity, Blending::NoBlend);

    stitcher->initTrafosFromMultipleVideos(sizes, localOptimalTrafos, optimalTransitions);
    auto panoImg0 = std::get<0>(stitcher->stitchPano(cv::Size(2*1920, 2*1080)));
    drawImg(panoImg0);

    return 0;
}
