#include <iostream>
#include <fstream>

#include "image-processing/images.h"
#include "panorama/keyFrames.h"
#include "cxxopts.hpp"

using namespace ht;
namespace fs = std::filesystem;


int main(int argc, char** argv)
{
    fs::path key_frames_file;
    bool absolute = false;

    cxxopts::Options options("keyframes_to_image_list", "");
    options.add_options()(
        "k,keyframes", "key_frames.yml used for translating", cxxopts::value(key_frames_file))(
        "a,absolute", "if paths should be written as absolute paths", cxxopts::value(absolute));


    auto result = options.parse(argc, argv);
    if (result.count("k") != 1 || key_frames_file.empty())
    {
        std::cout << options.help() << std::endl;
        return -1;
    }

    std::cout << "You called: " << std::endl;
    std::cout << "k: " << key_frames_file.string() << std::endl;
    std::cout << "a: " << absolute << std::endl;

    auto imgPath = key_frames_file.parent_path() / "imgs";
    auto imgs = Images(imgPath);
    auto kfs = KeyFrames::fromDir(key_frames_file);

    std::ofstream imgList("image_list.txt", std::ios::out);
    for (std::size_t i = 0; i < kfs.size(); i++)
    {
        auto imgFile = imgPath / imgs.getFileName(kfs[i]);
        if (absolute)
            imgFile = fs::absolute(imgFile);

        imgList << imgFile.string();
        if (i < kfs.size() - 1)
            imgList << "\n";
    }
}