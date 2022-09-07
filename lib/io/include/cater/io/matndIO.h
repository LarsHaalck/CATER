#ifndef CT_MAT_N_IO_H
#define CT_MAT_N_IO_H

#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/vector.hpp>

#include <cater/io/io.h>

#include <opencv2/core.hpp>

namespace ct
{
struct MatND
{
    cv::Mat _m;
    MatND(cv::Mat m) : _m(m) {}
    MatND() : _m() {}
};
}

namespace cereal
{
template <class Archive>
void save(Archive& archive, const ct::MatND& mat)
{
    int dims = mat._m.dims;
    int type = mat._m.type();
    archive(dims, type);

    int numElem = 1;
    for (int i = 0; i < dims; i++)
    {
        auto sz = mat._m.size[i];
        archive(sz);
        numElem *= sz;
    }

    int dataSize = numElem * static_cast<int>(mat._m.elemSize());
    archive(cereal::binary_data(mat._m.ptr(), dataSize));
}

template <class Archive>
void load(Archive& archive, ct::MatND& mat)
{
    int dims, type;
    archive(dims, type);

    int numElem = 1;
    std::vector<int> dimSizes;
    for (int i = 0; i < dims; i++)
    {
        int sz;
        archive(sz);
        dimSizes.push_back(sz);
        numElem *= sz;
    }

    mat._m.create(dims, dimSizes.data(), type);
    int dataSize = numElem * static_cast<int>(mat._m.elemSize());
    archive(cereal::binary_data(mat._m.ptr(), dataSize));
}
} // namespace cereal

#endif // CT_MAT_N_IO_H
