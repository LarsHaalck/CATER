#ifndef HT_MAT_IO_H
#define HT_MAT_IO_H

#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/vector.hpp>

#include "io/io.h"
#include <opencv2/core.hpp>

namespace cereal
{
template <class Archive>
void save(Archive& archive, const cv::Mat& mat)
{
    int rows = mat.rows;
    int cols = mat.cols;
    int type = mat.type();
    bool continuous = mat.isContinuous();

    archive(rows, cols, type, continuous);

    if (continuous)
    {
        // store as one continues array
        int dataSize = rows * cols * static_cast<int>(mat.elemSize());
        archive(cereal::binary_data(mat.ptr(), dataSize));
    }
    else
    {
        // store row-wise
        int rowSize = cols * static_cast<int>(mat.elemSize());
        for (int i = 0; i < rows; i++)
            archive(cereal::binary_data(mat.ptr(i), rowSize));
    }
}

template <class Archive>
void load(Archive& archive, cv::Mat& mat)
{
    int rows, cols, type;
    bool continuous;
    archive(rows, cols, type, continuous);

    if (continuous)
    {
        mat.create(rows, cols, type);
        int dataSize = rows * cols * static_cast<int>(mat.elemSize());
        archive(cereal::binary_data(mat.ptr(), dataSize));
    }
    else
    {
        mat.create(rows, cols, type);
        int rowSize = cols * static_cast<int>(mat.elemSize());
        for (int i = 0; i < rows; i++)
            archive(cereal::binary_data(mat.ptr(i), rowSize));
    }
}
} // namespace cereal

#endif // HT_MAT_IO_H
