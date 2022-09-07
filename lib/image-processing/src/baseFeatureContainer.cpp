#include <cater/image-processing/baseFeatureContainer.h>

namespace ct
{
std::ostream& operator<<(std::ostream& os, const FeatureType& type)
{
    switch (type)
    {
    case FeatureType::ORB:
        os << "ORB";
        break;
    case FeatureType::SIFT:
        os << "SIFT";
        break;
    default:
        os << "operator<< not defined for supplied FeatureType";
    }
    return os;
}
} // namespace ct
