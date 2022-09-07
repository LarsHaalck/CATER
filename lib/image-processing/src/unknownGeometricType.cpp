#include "unknownGeometricType.h"

namespace ct
{
UnknownGeometricType::UnknownGeometricType(const std::string& type)
    : std::exception()
    , mType(std::string("Unknown Geometric Type suffix ") + type)
{
}
const char* UnknownGeometricType::what() const throw() { return mType.c_str(); }

}
