#ifndef HABITRACK_UNKNOWN_GEOMETRIC_TYPE_EXCEPTION_H
#define HABITRACK_UNKNOWN_GEOMETRIC_TYPE_EXCEPTION_H

#include <exception>
#include <string>

namespace ht
{
class UnknownGeometricType : public std::exception
{
public:
    UnknownGeometricType(const std::string& type = std::string());
    const char* what() const throw();

private:
    std::string mType;
};
} // namespace ht

#endif // HABITRACK_UNKNOWN_GEOMETRIC_TYPE_EXCEPTION_H
