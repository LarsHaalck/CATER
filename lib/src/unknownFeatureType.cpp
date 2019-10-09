#include "habitrack/unknownFeatureType.h"

namespace ht
{
UnknownFeatureType::UnknownFeatureType(const std::string& type)
    : std::exception()
    , mType(type)
{
}
const char* UnknownFeatureType::what() const throw()
{
    return (std::string("Unknown Feature Type suffix ") + mType).c_str();
}

}
