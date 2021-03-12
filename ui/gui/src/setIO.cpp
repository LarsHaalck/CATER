#include "setIO.h"

#include "spdlog/spdlog.h"
#include <opencv2/core/persistence.hpp>

namespace gui
{
namespace fs = std::filesystem;

void saveSet(const std::filesystem::path& setFile, const std::unordered_set<std::size_t>& set)
{
    spdlog::info("Save set file to: {}", setFile.string());
    spdlog::debug("Saving {} elems in set file", set.size());
    cv::FileStorage fs(setFile.string(), cv::FileStorage::WRITE);
    std::vector<std::size_t> vec(std::begin(set), std::end(set));
    std::sort(std::begin(vec), std::end(vec));

    fs << "invisible"
       << "[";
    for (auto elem : vec)
        fs << static_cast<int>(elem);
    fs << "]";
}

std::unordered_set<std::size_t> loadSet(const std::filesystem::path& setFile)
{
    spdlog::info("Load set from: {}", setFile.string());
    cv::FileStorage fs(setFile.string(), cv::FileStorage::READ);

    std::unordered_set<std::size_t> set;
    auto node = fs["invisible"];
    for (cv::FileNodeIterator it = node.begin(); it != node.end(); ++it)
    {
        int elem = (*it);
        set.insert(elem);
    }
    spdlog::debug("Read {} elems from set file", set.size());
    return set;
}
} // namespace gui
