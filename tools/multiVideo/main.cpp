#include <cxxopts.hpp>

#include <habitrack/image-processing/images.h>
#include <habitrack/panorama/panoramaEngine.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>
#include <iostream>
#include <memory>
#include <numeric>

using namespace ht;
namespace fs = std::filesystem;

int main(int argc, const char** argv)
{
    /* spdlog::set_level(spdlog::level::debug); */

    std::vector<fs::path> videoPaths;
    int featureInt = 0;
    bool force = false;
    int cacheSize = 200;
    int cols = 2 * 1920;
    int rows = 2 * 1080;
    int stage = 0;

    cxxopts::Options options("multi", "");
    options.show_positional_help();
    options.add_options()("folders", "folders", cxxopts::value(videoPaths))(
        "g,feature", "feature type 0: ORB, 1: SIFT, 2: SuperPoint", cxxopts::value(featureInt))("f",
        "force", cxxopts::value(force))("k, cols", "cols", cxxopts::value(cols))("l, rows", "rows",
        cxxopts::value(rows))("s,stage", "0: init, 1: opt: 2: refined", cxxopts::value(stage));

    options.parse_positional({"folders"});

    auto result = options.parse(argc, argv);
    if (result.count("folders") < 2)
    {
        std::cout << options.help() << std::endl;
        return -1;
    }

    auto ftType = FeatureType::ORB;
    if (featureInt == 1)
        ftType = FeatureType::SIFT;
    if (featureInt == 2)
        ftType = FeatureType::SuperPoint;

    fs::path basePath = videoPaths[0].parent_path();
    if (basePath.empty())
        basePath = ".";

    PanoramaSettings settings;
    settings.force = force;
    settings.stage = static_cast<PanoramaStage>(stage);
    settings.cacheSize = cacheSize;
    settings.rows = rows;
    settings.cols = cols;
    settings.featureType = ftType;

    std::vector<Images> imgContainers;
    for (const auto& path : videoPaths)
    {
        auto images = Images(path / "imgs");
        imgContainers.push_back(images);
    }

    PanoramaEngine::runMulti(imgContainers, videoPaths, basePath, settings);

    return 0;
}
