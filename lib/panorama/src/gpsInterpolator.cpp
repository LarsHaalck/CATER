#include <habitrack/panorama/gpsInterpolator.h>

#include <habitrack/io/ptsIO.h>
#include <habitrack/io/io.h>

namespace ht
{
GPSInterpolator::GPSInterpolator(const GPSSettings& settings, std::size_t start_frame)
    : mSettings(settings)
    , mStartFrame(start_frame)
{
}

GPSMap GPSInterpolator::interpolate(const std::vector<std::size_t>& keyFrames) const
{
    auto gps = io::loadPoints<double>(mSettings.file);
    auto gps_frame_offset = mSettings.offset * mSettings.frame_sampling_rate;
    auto rate_ratio = static_cast<double>(mSettings.frame_sampling_rate) / mSettings.sampling_rate;

    auto filename = std::filesystem::path("gps_interp.csv");
    std::ofstream stream(filename.string());
    io::checkStream(stream, filename);

    std::unordered_map<std::size_t, cv::Point2d> gps_kf;
    for (auto kf : keyFrames)
    {
        auto real_frame = kf + mStartFrame;

        auto div = (real_frame - gps_frame_offset) / rate_ratio;
        auto prev = static_cast<int>(std::floor(div));
        auto next = prev + 1;

        if (prev >= 0 && static_cast<std::size_t>(next) < gps.size())
        {
            // prev < div < next, prev + 1 = next
            auto interp = (next - div) * gps[prev] + (div - prev) * gps[next];
            gps_kf[kf] = interp;

            stream << kf << "," << prev << "," << next << "\n";
        }
        else
            stream << kf << "," << -1 << "," << -1 << "\n";
    }

    return gps_kf;
}
} // ht
