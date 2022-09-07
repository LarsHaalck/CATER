#ifndef CATER_BASE_PROGRESS_BAR_H
#define CATER_BASE_PROGRESS_BAR_H

#include <string>

namespace ct
{
class BaseProgressBar
{
public:
    virtual void setTotal(std::size_t total) = 0;
    virtual void inc() = 0;
    virtual void inc(std::size_t inc) = 0;
    virtual void done() { }
    virtual void status(const std::string&) { }
    virtual ~BaseProgressBar() { }
};
} // namespace ct
#endif // CATER_BASE_PROGRESS_BAR_H
