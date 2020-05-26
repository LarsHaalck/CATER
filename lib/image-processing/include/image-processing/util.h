#ifndef HABITRACK_UTIL_H
#define HABITRACK_UTIL_H

#include <numeric>
#include <vector>

namespace ht
{
inline std::vector<std::size_t> getContinuousIds(std::size_t start, std::size_t end)
{
    std::vector<std::size_t> ids(end - start);
    std::iota(std::begin(ids), std::end(ids), 0);
    return ids;
}
} // namespace ht
#endif // HABITRACK_UTIL_H
