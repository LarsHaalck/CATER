#include <habitrack/image-processing/geometricType.h>

namespace ht
{
std::vector<GeometricType> typeToTypeList(GeometricType t)
{
    std::vector<GeometricType> types;
    if (static_cast<unsigned int>(t & GeometricType::Putative))
        types.push_back(GeometricType::Putative);
    if (static_cast<unsigned int>(t & GeometricType::Homography))
        types.push_back(GeometricType::Homography);
    if (static_cast<unsigned int>(t & GeometricType::Affinity))
        types.push_back(GeometricType::Affinity);
    if (static_cast<unsigned int>(t & GeometricType::Similarity))
        types.push_back(GeometricType::Similarity);
    if (static_cast<unsigned int>(t & GeometricType::Isometry))
        types.push_back(GeometricType::Isometry);
    return types;
}

std::ostream& operator<<(std::ostream& os, const GeometricType& type)
{
    switch (type)
    {
    case GeometricType::Undefined:
        os << "Undefined";
        break;
    case GeometricType::Putative:
        os << "Putative";
        break;
    case GeometricType::Homography:
        os << "Homography";
        break;
    case GeometricType::Affinity:
        os << "Affinity";
        break;
    case GeometricType::Similarity:
        os << "Similarity";
        break;
    case GeometricType::Isometry:
        os << "Isometry";
        break;
    default:
        break;
    }
    return os;
}
} // namespace ht
