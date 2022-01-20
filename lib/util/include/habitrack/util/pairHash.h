#ifndef HABITRACK_PAIR_HASH_H
#define HABITRACK_PAIR_HASH_H

#include <utility>

namespace ht
{
struct hash
{
    template <typename T, typename U>
    std::size_t operator()(const std::pair<T, U>& pair) const
    {
        return std::hash<T>()(pair.first) ^ std::hash<U>()(pair.second);
    }
};
} // namespace ht

#endif // HABITRACK_PAIR_HASH_H
