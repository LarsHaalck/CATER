#ifndef HT_PTS_IO_H
#define HT_PTS_IO_H

#include "io/io.h"
#include <filesystem>
#include <fstream>
#include <opencv2/core.hpp>

namespace ht::io
{
template <class T>
void savePoints(const std::filesystem::path& filename, const std::vector<cv::Point_<T>>& pts)
{
    std::ofstream stream(filename.string());
    checkStream(stream, filename);
    for (auto pt : pts)
        stream << pt.x << "," << pt.y << "\n";
}

} // namespace io

#endif // HT_PTS_IO_H
