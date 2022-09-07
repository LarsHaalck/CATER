#ifndef CT_PTS_IO_H
#define CT_PTS_IO_H

#include <cater/io/io.h>

#include <filesystem>
#include <fstream>
#include <opencv2/core.hpp>

namespace ct::io
{
template <class T>
void savePoints(const std::filesystem::path& filename, const std::vector<cv::Point_<T>>& pts)
{
    std::ofstream stream(filename.string());
    checkStream(stream, filename);
    for (auto pt : pts)
        stream << pt.x << "," << pt.y << "\n";
}

template <class T>
std::vector<cv::Point_<T>> loadPoints(const std::filesystem::path& filename)
{
    std::ifstream stream(filename.string());
    checkStream(stream, filename);
    std::vector<cv::Point_<T>> pts;
    for (std::string line; std::getline(stream, line);)
    {
        auto it = line.find(",");
        auto a = static_cast<T>(std::stod(line.substr(0, it)));
        auto b = static_cast<T>(std::stod(line.substr(it + 1)));
        pts.push_back({a, b});
    }
    return pts;
}

} // namespace io

#endif // CT_PTS_IO_H
