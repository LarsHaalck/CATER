#include <filesystem>
#include <iostream>
#include <memory>

#include <spdlog/spdlog.h>

#include "image-processing/features.h"
#include "image-processing/images.h"
#include "image-processing/matches.h"
#include "image-processing/mildRecommender.h"
#include "image-processing/superGlue.h"

#include "cxxopts.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace ht;
namespace fs = std::filesystem;

int main(int argc, const char** argv)
{
    spdlog::set_level(spdlog::level::debug);

    fs::path img1;
    fs::path img2;
    int targetWidth;

    cxxopts::Options options("superglue", "");
    options.show_positional_help();
    options.add_options()("a,img_1", "image 1", cxxopts::value(img1))("b,img_2", "image 2",
        cxxopts::value(img2))("w,target_width", "target width", cxxopts::value(targetWidth));

    options.parse_positional({"img_1", "img_2", "target_width", "rest"});
    auto result = options.parse(argc, argv);

    if (result.count("img_1") != 1 || result.count("img_2") != 1
        || result.count("target_width") != 1)
    {
        std::cout << options.help() << std::endl;
        return -1;
    }

    std::cout << img1.string() << std::endl;
    std::cout << img2.string() << std::endl;
    std::cout << targetWidth << std::endl;

    auto geomType = GeometricType::Similarity;
    auto images = Images({img1, img2});

    auto superglue
        = matches::SuperGlue("/home/lars/gitProjects/SuperGluePretrainedNetwork/cpp", targetWidth);
    superglue.compute(images, "fts", "mtch", geomType, matches::MatchType::Exhaustive);

    return 0;
}
