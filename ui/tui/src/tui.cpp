#include "tui.h"

#include <iostream>

constexpr char prompt[] = "HabiTrack >>> ";

namespace tui
{
Tui::Tui(int, char**) { }
int Tui::run()
{
    std::string response;
    int rc = 0;
    do
    {
        std::cout << prompt;
        getline(std::cin, response);
        rc = parse(response);
    } while (rc == 0 && (response != "exit" && !response.empty()));

    return rc;
}

int Tui::parse(const std::string& response)
{
    std::string cmd = response;
    std::string args = "";
    if (auto pos = response.find(" "); pos != std::string::npos)
    {
        cmd = response.substr(0, pos);
        args = response.substr(pos + 1);
    }

    if (cmd == "exit" || cmd.empty() || cmd == "#")
    {
        std::cout << std::endl;
        return 0;
    }
    else if (cmd == "open")
        openImages(args);
    else if (cmd == "load")
        loadResults(args);
    else if (cmd == "start")
        setStart(args);
    else if (cmd == "end")
        setEnd(args);
    else if (cmd == "save")
        save();
    else if (cmd == "prefs")
        prefs(args);
    else if (cmd == "extractFeatures" || cmd == "ef")
        extractFeatures();
    else if (cmd == "extractTrafos" || cmd == "et")
        extractTrafos();
    else if (cmd == "extractUnaries" || cmd == "eu")
        extractUnaries();
    else if (cmd == "optimize" || cmd == "opt")
        optimize();
    else if (cmd == "track")
        track();
    else
    {
        std::cout << "Unknown Command: " << cmd << "\n";
        return 0;
    }

    return 0;
}

void Tui::prefs(const std::string& args)
{
    if (args.empty())
    {
        std::cout << mHabiTrack.getPreferences() << std::endl;
        return;
    }

    auto currPrefs = mHabiTrack.getPreferences();

    std::vector<std::string> words;
    extractWords(args, std::back_inserter(words));

    for (std::size_t i = 0; i <= words.size() / 2; i += 2)
    {
        if (words[i] == "cacheSize")
            currPrefs.cacheSize = std::stoi(words[i + 1]);
        else if (words[i] == "chunkSize")
            currPrefs.chunkSize = std::stoi(words[i + 1]);
        else if (words[i] == "detectionRadius")
            currPrefs.detectionRadius = std::stoi(words[i + 1]);
        else if (words[i] == "fps")
            currPrefs.fps = std::stoi(words[i + 1]);
        else if (words[i] == "pixelsPerMm")
            currPrefs.pixelsPerMm = std::stoi(words[i + 1]);
        else if (words[i] == "featureType")
        {
            if (words[i + 1] == "ORB")
                currPrefs.featureType = ht::FeatureType::ORB;
            else if (words[i + 1] == "SIFT")
                currPrefs.featureType = ht::FeatureType::SIFT;
            else if (words[i + 1] == "SuperPoint")
                currPrefs.featureType = ht::FeatureType::SuperPoint;
        }
        else if (words[i] == "numFeatures")
            currPrefs.numFeatures = std::stoi(words[i + 1]);
        else if (words[i] == "unarySubsample")
            currPrefs.unarySubsample = std::stod(words[i + 1]);
        else if (words[i] == "unarySigma")
            currPrefs.unarySigma = std::stod(words[i + 1]);
        else if (words[i] == "removeRedLasers")
            currPrefs.removeRedLasers = std::stoi(words[i + 1]);
        else if (words[i] == "unarySuppress")
            currPrefs.unarySuppress = std::stod(words[i + 1]);
        else if (words[i] == "unaryMultiplier")
            currPrefs.unaryMultiplier = std::stod(words[i + 1]);
        else if (words[i] == "manualUnarySize")
            currPrefs.manualUnarySize = std::stoi(words[i + 1]);
        else if (words[i] == "pairwiseSize")
            currPrefs.pairwiseSize = std::stoi(words[i + 1]);
        else if (words[i] == "pairwiseSigma")
            currPrefs.pairwiseSigma = std::stod(words[i + 1]);
        else if (words[i] == "smoothBearing")
            currPrefs.smoothBearing = std::stoi(words[i + 1]);
        else if (words[i] == "smoothBearingWindowSize")
            currPrefs.smoothBearingWindowSize = std::stoi(words[i + 1]);
        else if (words[i] == "smoothBearingOutlierTol")
            currPrefs.smoothBearingOutlierTol = std::stoi(words[i + 1]);
        else if (words[i] == "removeCamMotion")
            currPrefs.removeCamMotion = std::stoi(words[i + 1]);
        else if (words[i] == "nnRatio")
            currPrefs.nnRatio = std::stod(words[i + 1]);
        else if (words[i] == "ranscacReproj")
            currPrefs.ranscacReproj = std::stod(words[i + 1]);
    }

    mHabiTrack.setPreferences(currPrefs);
}
} // namespace tui
