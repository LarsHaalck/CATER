#ifndef GUI_LABEL_IO_H
#define GUI_LABEL_IO_H

#include <string>
#include <vector>

#include <cereal/archives/json.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/string.hpp>

#include "io/io.h"

namespace cereal
{
//! Serializing for std::pair
template <class Archive>
void serialize(Archive& archive, std::pair<std::string, int>& labelId)
{
    archive(make_nvp("label", labelId.first));
    archive(make_nvp("value:", labelId.second));
}

template <class Archive, class T>
void serialize(Archive& archive, std::pair<std::size_t, T>& test)
{
    archive(make_nvp("frame", test.first));
    archive(make_nvp("labels", test.second));
}

template <class Archive, class T>
void save(Archive& archive, const std::vector<T>& vec)
{
    archive(make_size_tag(static_cast<size_type>(vec.size())));
    for (std::size_t i = 0; i < vec.size(); i++)
        archive(std::make_pair(i, vec[i]));
}

template <class Archive, class T>
void load(Archive& archive, std::vector<T>& vec)
{
    size_type size;
    archive(make_size_tag(size));
    vec.resize(static_cast<std::size_t>(size));

    for (auto&& v : vec)
    {
        auto pair = std::pair<std::size_t, T>();
        archive(pair);
        v = pair.second;
    }
}

} // namespace cereal
#endif // GUI_LABEL_IO_H
