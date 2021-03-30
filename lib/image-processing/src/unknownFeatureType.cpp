#include "unknownFeatureType.h"

namespace ht
{
UnknownFeatureType::UnknownFeatureType(const std::string& type)
    : std::exception()
    , mType(std::string("Unknown Feature Type suffix ") + type)
{
}
const char* UnknownFeatureType::what() const throw() { return mType.c_str(); }

}
