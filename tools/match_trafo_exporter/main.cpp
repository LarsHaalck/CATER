#include <fstream>
#include <iostream>

#include <habitrack/image-processing/features.h>
#include <habitrack/image-processing/matches.h>
#include <habitrack/panorama/keyFrames.h>

#include <cxxopts.hpp>

using namespace ht;
namespace fs = std::filesystem;

void exportMatches(fs::path file, const std::unordered_map<std::size_t, std::size_t>& kfs)
{
    auto matches = matches::getMatches(file);
    auto keys = matches::getKeyList(matches);

    std::ofstream csv(file.replace_extension(".csv"), std::ofstream::out);
    for (auto&& key : keys)
    {
        auto pair = key;
        if (!kfs.empty())
        {
            pair.first = kfs.at(pair.first);
            pair.second = kfs.at(pair.second);
        }
        csv << pair.first << "," << pair.second << "," << matches.at(key).size() << std::endl;
    }
}

void exportTrafos(fs::path file, const std::unordered_map<std::size_t, std::size_t>& kfs)
{
    auto trafos = matches::getTrafos(file);
    auto keys = matches::getKeyList(trafos);

    std::ofstream csv(file.replace_extension(".csv"), std::ofstream::out);
    for (auto&& key : keys)
    {
        auto pair = key;
        if (!kfs.empty())
        {
            pair.first = kfs.at(pair.first);
            pair.second = kfs.at(pair.second);
        }
        auto trafo = trafos.at(key);
        csv << pair.first << "," << pair.second << ",";
        for (int r = 0; r < trafo.rows; r++)
        {
            for (int c = 0; c < trafo.cols; c++)
            {
                if (r == trafo.rows - 1 && c == trafo.cols - 1)
                    csv << trafo.at<double>(r, c) << std::endl;
                else
                    csv << trafo.at<double>(r, c) << ",";
            }
        }
    }
}

int main(int argc, const char** argv)
{
    fs::path file;
    fs::path key_frames_file;

    cxxopts::Options options("single", "");
    options.add_options()("i,in", "matches or trafo file", cxxopts::value(file))(
        "k,keyframes", "key_frames.yml used for translating", cxxopts::value(key_frames_file));

    auto result = options.parse(argc, argv);
    if (result.count("in") != 1)
    {
        std::cout << options.help() << std::endl;
        return -1;
    }

    std::cout << "You called: " << std::endl;
    std::cout << "i: " << file.string() << std::endl;
    std::cout << "k: " << key_frames_file.string() << std::endl;

    std::unordered_map<std::size_t, std::size_t> keyframes;
    if (!key_frames_file.empty())
    {
        auto tmp = KeyFrames::fromDir(key_frames_file);
        for (std::size_t i = 0; i < tmp.size(); i++)
            keyframes.insert({tmp[i], i});
    }

    if (file.string().find("matches") != std::string::npos)
        exportMatches(file, keyframes);
    else if (file.string().find("trafos") != std::string::npos)
        exportTrafos(file, keyframes);
    else
    {
        std::cout << options.help() << std::endl;
        return -1;
    }
}
