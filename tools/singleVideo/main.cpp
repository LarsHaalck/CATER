#include <filesystem>
#include <iostream>
#include <memory>

#include <spdlog/spdlog.h>

#include "image-processing/images.h"
#include "panorama/panoramaEngine.h"

#include "cxxopts.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace ht;
namespace fs = std::filesystem;

int main(int argc, const char** argv)
{
    /* spdlog::set_level(spdlog::level::debug); */

    fs::path basePath;
    bool showResults = false;
    bool force = false;
    int stage = 0;
    std::size_t cacheSize = 200;
    int cols = 2 * 1920;
    int rows = 2 * 1080;
    int numFts = 500;
    double minCoverage = 0.0;
    int featureInt = 0;

    cxxopts::Options options("single", "");
    options.add_options()("i,in", "base path containing imgs dir", cxxopts::value(basePath))(
        "v", "show results", cxxopts::value(showResults))("c", "cache", cxxopts::value(cacheSize))(
        "f", "force", cxxopts::value(force))("k, cols", "cols", cxxopts::value(cols))(
        "l, rows", "rows", cxxopts::value(rows))("n,num_fts", "num", cxxopts::value(numFts))(
        "m, min_coverage", "min coverage", cxxopts::value(minCoverage))(
        "g,feature", "feature type 0: ORB, 1: SIFT, 2: SuperPoint", cxxopts::value(featureInt))(
        "s,stage", "0: init, 1: opt: 2: refined", cxxopts::value(stage));

    auto result = options.parse(argc, argv);
    if (result.count("in") != 1)
    {
        std::cout << options.help() << std::endl;
        return -1;
    }

    std::cout << "You called: " << std::endl;
    std::cout << "i: " << basePath.string() << std::endl;
    std::cout << "v: " << showResults << std::endl;
    std::cout << "c: " << cacheSize << std::endl;
    std::cout << "k: " << cols << std::endl;
    std::cout << "k: " << rows << std::endl;
    std::cout << "n: " << numFts << std::endl;
    std::cout << "m: " << minCoverage << std::endl;
    std::cout << "g: " << featureInt << std::endl;
    std::cout << "s: " << stage << std::endl;

    auto ftType = FeatureType::ORB;
    if (featureInt == 1)
        ftType = FeatureType::SIFT;
    if (featureInt == 2)
        ftType = FeatureType::SuperPoint;

    PanoramaSettings settings;
    settings.force = force;
    settings.stage = stage;
    settings.cacheSize = cacheSize;
    settings.rows = rows;
    settings.cols = cols;
    settings.numFts = numFts;
    settings.ftType = ftType;

    // load images
    auto images = Images(basePath / "imgs");
    PanoramaEngine::stitchPano(images, basePath, settings);

    return 0;
}
