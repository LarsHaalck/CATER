#include "panorama/panoramaEngine.h"

#include "image-processing/featureAggregator.h"
#include "image-processing/features.h"
#include "image-processing/imageAggregator.h"
#include "image-processing/images.h"
#include "image-processing/matches.h"
#include "image-processing/mildRecommender.h"
#include "image-processing/superGlue.h"
#include "panorama/idTranslator.h"
#include "panorama/keyFrameRecommender.h"
#include "panorama/keyFrames.h"
#include "panorama/panoramaStitcher.h"
#include "panorama/transitions.h"

#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

namespace ht
{

namespace PanoramaEngine
{
    auto ORB = ht::FeatureType::ORB;
    auto SIM = ht::GeometricType::Similarity;

    void runSingle(const Images& images, const std::filesystem::path& dataFolder,
        const PanoramaSettings& settings, std::shared_ptr<BaseProgressBar> mBar)
    {
        auto basePath = dataFolder;
        fs::create_directories(basePath);

        // calc features for all frames
        auto ftPath = basePath / "fts";
        auto features = Features();
        if (Features::isComputed(images, ftPath, ORB) && !settings.force)
            features = Features::fromDir(images, ftPath, ORB);
        else
            features = Features::compute(
                images, ftPath, ORB, settings.numFts, settings.cacheSize, size_t_vec(), mBar);

        // select key frames
        auto kfPath = basePath / "key_frames.yml";
        std::vector<std::size_t> keyFrames;
        if (KeyFrames::isComputed(kfPath) && !settings.force)
            keyFrames = KeyFrames::fromDir(kfPath);
        else
            keyFrames = KeyFrames::compute(features, SIM, kfPath, 0.3, 0.5, mBar);

        // calculate matches between keyframes and intermediate frames via KeyFrameRecommender
        auto kfIntraPath = basePath / "kfs/maches_intra";
        auto kfRecommender = std::make_unique<KeyFrameRecommender>(keyFrames);
        if (!matches::isComputed(kfIntraPath, SIM) || settings.force)
        {
            matches::compute(kfIntraPath, SIM, features, matches::MatchType::Strategy, 0, 0.0,
                std::move(kfRecommender), settings.cacheSize, size_t_vec(), mBar);
        }

        // calculate matches between keyframes via exhaustive matching
        auto kfInterPath = basePath / "kfs/maches_inter";
        auto featuresDensePath = basePath / "kfs/fts";

        auto featuresDense = Features();
        if (Features::isComputed(images, featuresDensePath, ORB, keyFrames) && !settings.force)
            featuresDense = Features::fromDir(images, featuresDensePath, ORB, keyFrames);
        else
        {
            featuresDense = Features::compute(images, featuresDensePath, ORB, 4 * settings.numFts,
                settings.cacheSize, keyFrames, mBar);
        }

        // window is 4 because ceil(1 / 0.3) seems like a sensible default
        auto mildRecommender = std::make_unique<MildRecommender>(featuresDense, 1, true);
        if (settings.ftType == FeatureType::SuperPoint)
        {
            if (Features::isComputed(images, featuresDensePath, settings.ftType, keyFrames)
                && !settings.force)
            {
                featuresDense
                    = Features::fromDir(images, featuresDensePath, settings.ftType, keyFrames);
            }
            else
            {
                featuresDense
                    = matches::SuperGlue("/data/arbeit/sg/indoor", 800)
                          .compute(images, featuresDensePath, kfInterPath, SIM,
                              matches::MatchType::Strategy, 4, 0.0, std::move(mildRecommender),
                              settings.cacheSize, keyFrames, mBar);
            }
        }
        else if (settings.ftType == FeatureType::SIFT)
        {
            auto featuresSift = Features();
            if (Features::isComputed(images, featuresDensePath, settings.ftType, keyFrames)
                && !settings.force)
            {
                featuresSift
                    = Features::fromDir(images, featuresDensePath, settings.ftType, keyFrames);
            }
            else
            {
                featuresSift = Features::compute(images, featuresDensePath, settings.ftType,
                    4 * settings.numFts, settings.cacheSize, keyFrames, mBar);

                if (!matches::isComputed(kfInterPath, SIM) || settings.force)
                {
                    matches::compute(kfInterPath, SIM, featuresSift, matches::MatchType::Strategy,
                        4, 0.0, std::move(mildRecommender), settings.cacheSize, keyFrames, mBar);
                }
            }
            featuresDense = std::move(featuresSift);
        }
        else
        {
            if (!matches::isComputed(kfInterPath, SIM) || settings.force)
            {
                matches::compute(kfInterPath, SIM, featuresDense, matches::MatchType::Strategy, 4,
                    0.0, std::move(mildRecommender), settings.cacheSize, keyFrames, mBar);
            }
        }

        // panoramas can only be calculated from theses Types
        GeometricType useableTypes = matches::getConnectedTypes(kfInterPath, SIM, keyFrames);
        auto typeList = typeToTypeList(useableTypes);
        for (auto type : typeList)
            spdlog::info("Usable type for PanoramaSticher: {}", type);

        auto stitcher = PanoramaStitcher(images, keyFrames, SIM);

        // init trafos of keyframes by concatenating them
        stitcher.initTrafos(matches::getTrafos(kfInterPath, SIM));
        auto pano = std::get<0>(
            stitcher.stitchPano(cv::Size(settings.cols, settings.rows), false, {}, mBar));
        cv::imwrite((basePath / "pano0_init.png").string(), pano);

        if (settings.stage < 1)
            return;

        // globally optimized these keyframe tranformations and write them for later IVLC
        if (fs::exists(basePath / "kfs/opt_trafos.bin") && !settings.force)
            stitcher.loadTrafos(basePath / "kfs/opt_trafos.bin");
        else
        {
            stitcher.globalOptimizeKeyFrames(
                featuresDense, matches::getMatches(kfInterPath, SIM), 0, mBar);
            stitcher.writeTrafos(basePath / "kfs/opt_trafos.bin");
        }
        // reintegrate by geodesic interpolation of frames between keyframes
        stitcher.reintegrate();

        pano = std::get<0>(stitcher.stitchPano(cv::Size(settings.cols, settings.rows), false,
            basePath / "opt_centers_sparse.yml", mBar));
        cv::imwrite((basePath / "pano1_opt_sparse.png").string(), pano);

        if (settings.stage < 2)
            return;

        // refine all keyframes
        if (fs::exists(basePath / "opt_trafos.bin") && !settings.force)
            stitcher.loadTrafos(basePath / "opt_trafos.bin");
        else
        {
            stitcher.refineNonKeyFrames(features, matches::getMatches(kfIntraPath, SIM), 50, mBar);
            stitcher.writeTrafos(basePath / "opt_trafos.bin");
            if (settings.writeReadable)
                stitcher.writeTrafos(basePath / "opt_trafos.yml", WriteType::Readable);
        }
        pano = std::get<0>(stitcher.stitchPano(cv::Size(settings.cols, settings.rows), false,
            basePath / "opt_centers_dense.yml", mBar));
        cv::imwrite((basePath / "pano2_opt_dense.png").string(), pano);
    }

    void runMulti(const std::vector<Images>& imgContainers,
        const std::vector<std::filesystem::path>& dataFolders,
        const std::filesystem::path& outFolder, const PanoramaSettings& settings,
        std::shared_ptr<BaseProgressBar> mBar)
    {
        std::vector<std::vector<std::size_t>> keyFrameList;
        std::vector<std::size_t> sizes;
        std::vector<PairwiseMatches> matchesIntraList;
        std::vector<PairwiseMatches> matchesInterList;

        std::vector<Features> ftsOrbSparse;
        std::vector<Features> ftsOrbDense;
        std::vector<Features> ftsDense;

        std::vector<std::vector<cv::Mat>> localOptimalTrafos;

        auto basePath = outFolder;
        fs::create_directories(basePath);

        for (std::size_t i = 0; i < imgContainers.size(); i++)
        {
            auto path = dataFolders[i];
            const auto& images = imgContainers[i];
            auto currFtsOrbSparse = Features::fromDir(images, path / "fts", ORB);
            auto currFtsOrbDense = Features::fromDir(images, path / "kfs/fts", ORB);

            if (settings.ftType != ORB)
                ftsDense.push_back(Features::fromDir(images, path / "kfs/fts", settings.ftType));

            auto keyFrames = KeyFrames::fromDir(path / "key_frames.yml");
            keyFrameList.push_back(keyFrames);

            auto matchesIntra = matches::getMatches(path / "kfs/maches_intra", SIM);
            matchesIntraList.push_back(std::move(matchesIntra));

            auto matchesInter = matches::getMatches(path / "kfs/maches_inter", SIM);
            matchesInterList.push_back(std::move(matchesInter));

            ftsOrbSparse.push_back(std::move(currFtsOrbSparse));
            ftsOrbDense.push_back(std::move(currFtsOrbDense));

            sizes.push_back(images.size());
            localOptimalTrafos.push_back(PanoramaStitcher::getTrafos(path / "kfs/opt_trafos.bin"));
        }
        Translator translator(sizes);

        // aggregate img and feature container
        auto combinedImgContainer = ImageAggregator(imgContainers);
        auto combinedSparseFtContainer = FeatureAggregator(ftsOrbSparse);
        auto combinedDenseOrbFtContainer = FeatureAggregator(ftsOrbDense);
        auto combinedDenseFtContainer = FeatureAggregator(ftsDense);

        // list of local keyframes to global keyframes
        auto globalKeyFrames = translator.localToGlobal(keyFrameList);

        // match inter video
        auto ivlcMatchPath = basePath / "ivlc/matches";
        auto mildRecommender = std::make_unique<MildRecommender>(combinedDenseOrbFtContainer);
        if (!matches::isComputed(ivlcMatchPath, SIM) || settings.force)
        {
            if (settings.ftType != ORB)
            {
                matches::compute(ivlcMatchPath, SIM, combinedDenseFtContainer,
                    matches::MatchType::Strategy, 10, 0.0, std::move(mildRecommender),
                    settings.cacheSize, globalKeyFrames);
            }
            else
            {
                matches::compute(ivlcMatchPath, SIM, combinedDenseOrbFtContainer,
                    matches::MatchType::Strategy, 10, 0.0, std::move(mildRecommender),
                    settings.cacheSize, globalKeyFrames);
            }
        }

        // combined local dense matches and inter video sparse matches
        auto interVidMatches = matches::getMatches(ivlcMatchPath, SIM);
        auto optimalTransitions = getMostProminantTransition(interVidMatches, sizes);

        auto globalInterMatches = translator.localToGlobal(matchesInterList);
        globalInterMatches.insert(std::begin(interVidMatches), std::end(interVidMatches));

        auto stitcher = PanoramaStitcher(combinedImgContainer, globalKeyFrames, SIM);
        stitcher.initTrafosFromMultipleVideos(
            matches::getTrafos(ivlcMatchPath, SIM), sizes, localOptimalTrafos, optimalTransitions);

        auto pano = std::get<0>(
            stitcher.stitchPano(cv::Size(settings.cols, settings.rows), false, {}, mBar));
        cv::imwrite(basePath / "combined0_init.png", pano);

        if (settings.stage < 1)
            return;

        if (fs::exists(basePath / "opt_trafos_sparse.bin") && !settings.force)
            stitcher.loadTrafos(basePath / "opt_trafos_sparse.bin");
        else
        {
            if (settings.ftType != ORB)
                stitcher.globalOptimizeKeyFrames(
                    combinedDenseFtContainer, globalInterMatches, 0, mBar);
            else
                stitcher.globalOptimizeKeyFrames(
                    combinedDenseOrbFtContainer, globalInterMatches, 0, mBar);
            stitcher.writeTrafos(basePath / "opt_trafos_sparse.bin");
        }

        stitcher.reintegrate();
        pano = std::get<0>(stitcher.stitchPano(cv::Size(settings.cols, settings.rows), false,
            basePath / "opt_centers_sparse.yml", mBar));
        cv::imwrite(basePath / "combined1_opt_sparse.png", pano);

        if (settings.stage < 2)
            return;

        if (fs::exists(basePath / "opt_trafos.bin") && !settings.force)
            stitcher.loadTrafos(basePath / "opt_trafos.bin");
        else
        {
            auto globalIntraMatches = translator.localToGlobal(matchesIntraList);
            stitcher.refineNonKeyFrames(combinedSparseFtContainer, globalIntraMatches, 0, mBar);
            stitcher.writeTrafos(basePath / "opt_trafos.bin");
            if (settings.writeReadable)
                stitcher.writeTrafos(basePath / "opt_trafos.yml", WriteType::Readable);
        }

        pano = std::get<0>(stitcher.stitchPano(cv::Size(settings.cols, settings.rows), false,
            basePath / "opt_centers_dense.yml", mBar));
        cv::imwrite(basePath / "combined2_opt_dense.png", pano);
    }

} // namespace PanoramaEngine
} // namespace ht
