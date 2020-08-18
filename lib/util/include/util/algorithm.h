#ifndef HT_UTIL_H
#define HT_UTIL_H

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace ht
{
template <typename InputIt>
auto mean(InputIt first, InputIt last) -> typename InputIt::value_type
{
    using T = typename InputIt::value_type;
    T sum = std::accumulate(first, last, static_cast<T>(0));
    return sum / static_cast<T>(std::distance(first, last));
}

template <typename InputIt>
auto std_dev(InputIt first, InputIt last) -> typename InputIt::value_type
{
    using T = typename InputIt::value_type;
    auto size = std::distance(first, last);

    T vec_mean = mean(first, last);
    std::vector<T> diff(size);
    std::transform(first, last, std::begin(diff), std::bind2nd(std::minus<T>(), vec_mean));

    T sq_sum
        = std::inner_product(std::begin(diff), std::end(diff), std::begin(diff), static_cast<T>(0));
    return std::sqrt(sq_sum / static_cast<T>(size));
}

inline double degree2Radian(double degree) { return ((degree * M_PI) / 180.0); }
inline double radian2Degree(double radian) { return ((radian * 180.0) / M_PI); }
} // namespace ht

#endif // HT_UTIL_H
