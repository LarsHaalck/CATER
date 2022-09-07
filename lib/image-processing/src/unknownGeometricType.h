#ifndef CATER_UNKNOWN_GEOMETRIC_TYPE_EXCEPTION_H
#define CATER_UNKNOWN_GEOMETRIC_TYPE_EXCEPTION_H

#include <exception>
#include <string>

namespace ct
{
class UnknownGeometricType : public std::exception
{
public:
    UnknownGeometricType(const std::string& type = std::string());
    const char* what() const throw();

private:
    std::string mType;
};
} // namespace ct

#endif // CATER_UNKNOWN_GEOMETRIC_TYPE_EXCEPTION_H
