#include "gui/mainWindow.h"
#include <QApplication>
#include <spdlog/spdlog.h>

#include <cxxopts.hpp>

int main(int argc, char* argv[])
{
    spdlog::set_level(spdlog::level::debug);

    QApplication a(argc, argv);
    gui::MainWindow w;
    w.show();
    return a.exec();
}