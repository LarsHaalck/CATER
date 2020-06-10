#include "gui/mainWindow.h"
#include <QApplication>
#include <spdlog/spdlog.h>

int main(int argc, char* argv[])
{
    /* spdlog::set_level(spdlog::level::debug); */
    QApplication a(argc, argv);
    gui::HabiTrack w;
    w.show();
    return a.exec();
}
