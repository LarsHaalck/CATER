#include <cater/progressbar/progressBar.h>

#include <iostream>

namespace ct
{
ProgressBar::ProgressBar()
    : mTotalTicks(100)
{
}

void ProgressBar::display() const
{
    float progress;
    if (mTotalTicks > 0)
        progress = static_cast<float>(mTicks) / mTotalTicks;
    else
        progress = 1.0f;

    auto pos = static_cast<std::size_t>(mBarWidth * progress);

    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    auto time_elapsed
        = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();

    std::cout << "[";

    for (std::size_t i = 0; i < mBarWidth; ++i)
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

void ProgressBar::status(const std::string&) { }
void ProgressBar::done()
{
    if (!mIsFinished)
    {
        mTicks += mTotalTicks - mTicks;
        display();
        std::cout << std::endl;
        mIsFinished = true;
    }
}
} // namespace ct
