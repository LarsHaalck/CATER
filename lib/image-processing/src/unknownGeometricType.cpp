#include "unknownGeometricType.h"

namespace ht
{
UnknownGeometricType::UnknownGeometricType(const std::string& type)
    : std::exception()
    , mType(std::string("Unknown Geometric Type suffix ") + type)
{
}
const char* UnknownGeometricType::what() const throw()
{
    return mType.c_str();
}

}
