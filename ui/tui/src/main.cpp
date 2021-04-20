#include <spdlog/spdlog.h>

#include "tui.h"

int main(int argc, char* argv[])
{
    spdlog::set_level(spdlog::level::debug);

    tui::Tui tui(argc, argv);
    return tui.run();
}
