#ifndef HABITRACK_GEOMETRIC_TYPE_H
#define HABITRACK_GEOMETRIC_TYPE_H

namespace ht
{
enum class GeometricType : unsigned int
{
    Undefined = 0x0,
    Isometry = 0x1,
    Similarity = 0x2,
    Affinity = 0x4,
    Homography = 0x8,
    Putative = 0x10,
};

inline constexpr GeometricType operator&(GeometricType x, GeometricType y)
{
    return static_cast<GeometricType>(
        static_cast<unsigned int>(x) & static_cast<unsigned int>(y));
}

inline constexpr GeometricType
operator|(GeometricType x, GeometricType y)
{
    return static_cast<GeometricType>(
        static_cast<unsigned int>(x) | static_cast<unsigned int>(y));
}

inline constexpr GeometricType
operator^(GeometricType x, GeometricType y)
{
    return static_cast<GeometricType>(
        static_cast<unsigned int>(x) ^ static_cast<unsigned int>(y));
}

inline constexpr GeometricType
operator~(GeometricType x)
{
    return static_cast<GeometricType>(~static_cast<unsigned int>(x));
}

inline GeometricType &
operator&=(GeometricType & x, GeometricType y)
{
    x = x & y;
    return x;
}

inline GeometricType &
operator|=(GeometricType & x, GeometricType y)
{
    x = x | y;
    return x;
}

inline GeometricType &
operator^=(GeometricType & x, GeometricType y)
{
    x = x ^ y;
    return x;
}
} // namespace ht
#endif // HABITRACK_GEOMETTRIC_TYPE_H

