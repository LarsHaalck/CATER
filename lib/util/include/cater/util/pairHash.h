#ifndef CATER_PAIR_HASH_H
#define CATER_PAIR_HASH_H

#include <utility>

namespace ct
{
struct hash
{
    template <typename T, typename U>
    std::size_t operator()(const std::pair<T, U>& pair) const
    {
        return std::hash<T>()(pair.first) ^ std::hash<U>()(pair.second);
    }
};
} // namespace ct

#endif // CATER_PAIR_HASH_H
