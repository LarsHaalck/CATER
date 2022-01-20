#ifndef PROGRESS_BAR_H
#define PROGRESS_BAR_H

#include <habitrack/progressbar/baseProgressBar.h>
#include <chrono>

namespace ht
{
class ProgressBar : public BaseProgressBar
{
public:
    ProgressBar();

    inline void setTotal(std::size_t total) override
    {
        mTicks = 0;
        mTotalTicks = total;
    }
    inline void inc() override
    {
        mTicks++;
        display();
    }

    inline void inc(std::size_t inc) override
    {
        mTicks += inc;
        display();
    }

    void status(const std::string& state) override;

    ~ProgressBar() { done(); }
    void done() override;

private:
    void display() const;

private:
    std::size_t mTicks = 0;

    std::size_t mTotalTicks;
    const std::size_t mBarWidth = 60;
    const char mCompleteChar = '=';
    const char mInCompleteChar = ' ';
    const std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    bool mIsFinished = false;
};
} // namespace ht
#endif // PROGRESS_BAR_H
