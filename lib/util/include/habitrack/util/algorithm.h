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

// copy by value, so we can sort in-place
template <typename T>
T median(std::vector<T> vec)
{
    using sizeT = typename std::vector<T>::size_type;
    sizeT size = vec.size();
    if (size == 0)
        return 0;

    std::sort(std::begin(vec), std::end(vec));

    if (size % 2 == 0)
        return (vec[size / 2 - 1] + vec[size / 2]) / 2;
    else
        return vec[size / 2];
}

// nth_element modifies in unpredictable ways
template <typename InputIt>
auto median_fast(InputIt first, InputIt last) -> typename InputIt::value_type
{
    auto size = std::distance(first, last);
    auto half = size / 2;
    std::nth_element(first, first + half, last);
    auto med = first + half;

    if (size % 2 == 0)
    {
        auto max_it = max_element(first, first + half);
        return (*med + *max_it) / 2;
    }
    return (*med);
}

template <typename InputIt>
auto std_dev(InputIt first, InputIt last) -> typename InputIt::value_type
{
    using T = typename InputIt::value_type;
    auto size = std::distance(first, last);

    T vec_mean = mean(first, last);
    std::vector<T> diff(size);
    std::transform(first, last, std::begin(diff), [vec_mean](auto val) { return vec_mean - val; });

    T sq_sum
        = std::inner_product(std::begin(diff), std::end(diff), std::begin(diff), static_cast<T>(0));
    return std::sqrt(sq_sum / static_cast<T>(size));
}

inline double degree2Radian(double degree) { return ((degree * M_PI) / 180.0); }
inline double radian2Degree(double radian) { return ((radian * 180.0) / M_PI); }

template <typename T>
std::vector<T> permute(const std::vector<T>& vec, const std::vector<std::size_t>& p)
{
    if (vec.empty())
        return vec;

    std::vector<T> sorted(p.size());
    std::transform(p.begin(), p.end(), sorted.begin(), [&](std::size_t i) { return vec[i]; });
    return sorted;
}
} // namespace ht

#endif // HT_UTIL_H
