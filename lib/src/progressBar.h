#ifndef PROGRESS_BAR_H
#define PROGRESS_BAR_H

#include <chrono>
#include <iostream>

namespace ht
{
class ProgressBar
{
public:
    ProgressBar(unsigned int total, unsigned int width)
        : mTotalTicks(total)
        , mBarWidth(width)
    {
    }

    ProgressBar(unsigned int total)
        : mTotalTicks(total)
    {
    }

    void operator++() { mTicks++; }
    ProgressBar& operator+=(int inc) { mTicks += inc; return *this; }

    void display() const
    {
        auto progress = static_cast<float>(mTicks) / mTotalTicks;
        auto pos = static_cast<unsigned int>(mBarWidth * progress);

        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        auto time_elapsed
            = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time)
                  .count();

        std::cout << "[";

        for (unsigned int i = 0; i < mBarWidth; ++i)
        {
            if (i < pos)
                std::cout << mCompleteChar;
            else if (i == pos)
                std::cout << ">";
            else
                std::cout << mInCompleteChar;
        }
        std::cout << "] " << static_cast<int>(progress * 100.0) << "% "
            << static_cast<float>(time_elapsed) / 1000.0 << "s\r";
        std::cout.flush();
    }

    ~ProgressBar()
    {
        done();
    }

    void done()
    {
        if (!mIsFinished)
        {
            mTicks += mTotalTicks - mTicks;
            display();
            std::cout << std::endl;
            mIsFinished = true;
        }
    }

private:
    unsigned int mTicks = 0;

    const unsigned int mTotalTicks;
    const unsigned int mBarWidth = 60;
    const char mCompleteChar = '=';
    const char mInCompleteChar = ' ';
    const std::chrono::steady_clock::time_point start_time
        = std::chrono::steady_clock::now();

    bool mIsFinished = false;

};
} // namespace ht
#endif // PROGRESS_BAR_H
