#ifndef CATER_UNKNOWN_FEATURE_TYPE_EXCEPTION_H
#define CATER_UNKNOWN_FEATURE_TYPE_EXCEPTION_H

#include <exception>
#include <string>

namespace ct
{
class UnknownFeatureType : public std::exception
{
public:
    UnknownFeatureType(const std::string& type = std::string());
    const char* what() const throw();

private:
    std::string mType;
};
} // namespace ct

#endif // CATER_UNKNOWN_FEATURE_TYPE_EXCEPTION_H
