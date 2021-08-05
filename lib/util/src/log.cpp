#include "util/log.h"

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <iostream>

namespace ht
{
void setLogFileTo(const std::filesystem::path& file)
{
    try
    {
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(spdlog::level::info);

        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file.string(), false);
        file_sink->set_level(spdlog::level::debug);

        auto logger = std::make_shared<spdlog::logger>(
            "multi_sink", spdlog::sinks_init_list({console_sink, file_sink}));
        logger->set_level(spdlog::level::debug);
        spdlog::flush_every(std::chrono::seconds(3));
        spdlog::set_default_logger(logger);
        spdlog::info("New run -------------");
    }
    catch (const spdlog::spdlog_ex& ex)
    {
        std::cerr << "Log initialization failed: " << ex.what();
    }
}

} // namespace ht
