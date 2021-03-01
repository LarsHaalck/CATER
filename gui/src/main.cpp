#include "gui/mainWindow.h"
#include <QApplication>
#include <spdlog/spdlog.h>

#include <cxxopts.hpp>

int main(int argc, char* argv[])
{
    spdlog::set_level(spdlog::level::debug);

    bool hideGui = false;
    std::filesystem::path imgs;
    std::filesystem::path resFile;
    cxxopts::Options options("habitrack", "");
    options.add_options()
        ("f", "hide gui (for terminal use)", cxxopts::value(hideGui))
        ("i,in", "imgs dir", cxxopts::value(imgs))
        ("r,res", "res file", cxxopts::value(imgs)) ;

    auto result = options.parse(argc, argv);

    // if gui is hidden but no image or res file is supplied
    if (result.count("f") == 1 && (result.count("in") != 1 && result.count("res") != 1))
    {
        std::cout << options.help() << std::endl;
        return -1;
    }

    std::cout << "You called: " << std::endl;
    std::cout << "f: " << hideGui << std::endl;
    std::cout << "i: " << imgs.string() << std::endl;
    std::cout << "r: " << resFile.string() << std::endl;

    int qNumArgs = 1;
    std::vector<std::string> qArgsVec;
    qArgsVec.push_back(argv[0]);

    if (hideGui)
    {
        qNumArgs += 2;
        qArgsVec.push_back("-platform");
        qArgsVec.push_back("offscreen");
    }

    std::vector<char*> qArgs;
    for (auto& s: qArgsVec)
        qArgs.push_back(&s[0]);

    for (int i = 0; i < qNumArgs; i++)
        std::cout << qArgs[i] << std::endl;

    QApplication a(qNumArgs, qArgs.data());
    gui::HabiTrack w;
    if (hideGui)
    {
        if (!resFile.empty())
            w.loadResultsFile(resFile);
        else
            w.loadImageFolder(imgs);

        w.runFullPipeline();
        w.save();

    }
    w.show();
    return a.exec();
}
