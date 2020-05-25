#ifndef HABITRACK_BASE_PROGRESS_BAR_H
#define HABITRACK_BASE_PROGRESS_BAR_H

#include <string>

namespace ht
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
} // namespace ht
#endif // HABITRACK_BASE_PROGRESS_BAR_H
