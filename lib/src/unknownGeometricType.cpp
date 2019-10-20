#include "unknownGeometricType.h"

namespace ht
{
UnknownGeometricType::UnknownGeometricType(const std::string& type)
    : std::exception()
    , mType(type)
{
}
const char* UnknownGeometricType::what() const throw()
{
    return (std::string("Unknown Geometric Type suffix ") + mType).c_str();
}

}
