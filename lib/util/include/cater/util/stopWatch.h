#include <atomic>
#include <chrono>

namespace ct
{
template <typename Clock = std::chrono::high_resolution_clock>
class StopWatch
{
    const typename Clock::time_point start_point;

public:
    StopWatch()
        : start_point(Clock::now())
    {
    }

    template <typename Rep = typename Clock::duration::rep,
        typename Units = typename Clock::duration>
    Rep elapsed_time() const
    {
        std::atomic_thread_fence(std::memory_order_relaxed);
        auto counted_time = std::chrono::duration_cast<Units>(Clock::now() - start_point).count();
        std::atomic_thread_fence(std::memory_order_relaxed);
        return static_cast<Rep>(counted_time);
    }
};

using PreciseStopWatch = StopWatch<>;
using MonotonicStopwatch = StopWatch<std::chrono::steady_clock>;
} // namespace ct
