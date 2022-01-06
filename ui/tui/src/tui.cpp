#include "tui.h"

#include "panorama/gpsInterpolator.h"
#include <iostream>
#include <spdlog/spdlog.h>

constexpr char prompt[] = "HabiTrack >>> ";

namespace fs = std::filesystem;

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
    else if (cmd == "video")
        video(args);
    else if (cmd == "pano_add")
        addPanorama(args);
    else if (cmd == "pano_list")
        listPanorama();
    else if (cmd == "pano_gen")
        generatePanorama();
    else if (cmd == "pano_prefs")
        panoramaPrefs(args);
    else
    {
        std::cout << "Unknown Command: " << cmd << "\n";
        return 0;
    }

    return 0;
}

void Tui::loadResults(const std::filesystem::path& resFile)
{
    mHabiTrack.loadResultsFile(resFile);
    addPanorama(resFile);
}

void Tui::prefs(const std::string& args)
{
    std::vector<std::string> words;
    extractWords(args, std::back_inserter(words));

    if (words.empty() || words.size() % 2 != 0)
    {
        std::cout << mHabiTrack.getPreferences() << std::endl;
        return;
    }

    auto currPrefs = mHabiTrack.getPreferences();
    for (std::size_t i = 0; i < words.size(); i += 2)
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

void Tui::addPanorama(const std::string& args)
{
    std::vector<std::string> words;
    extractWords(args, std::back_inserter(words));
    if (words.size() == 0)
    {
        std::cout << "Need to pass at least one argument (file [, gps])" << std::endl;
        return;
    }

    auto panoFile = words[0];
    std::string gpsFile;
    if (words.size() > 1)
        gpsFile = words[1];

    if (std::find(std::begin(mPanoFiles), std::end(mPanoFiles), panoFile) != std::end(mPanoFiles))
        return;

    if (!fs::is_regular_file(panoFile) || (!gpsFile.empty() && !fs::is_regular_file(gpsFile)))
    {
        std::cout << "Passed argument is not a file" << std::endl;
        return;
    }

    mPanoFiles.push_back(panoFile);
    if (!gpsFile.empty())
        mPanoGPSFiles[panoFile] = gpsFile;
}

void Tui::listPanorama()
{
    std::cout << "Added results files: \n";
    for (const auto& f : mPanoFiles)
        std::cout << f << std::endl;
}

void Tui::generatePanorama()
{
    std::vector<ht::Images> images;
    std::vector<fs::path> data;

    std::vector<cv::Point> pts;
    std::vector<std::size_t> chunkSizes;
    std::vector<ht::GPSInterpolator> gpsInterpolators;
    for (auto resFile : mPanoFiles)
    {
        ht::HabiTrack habitrack;
        habitrack.loadResultsFile(resFile);
        ht::GPSSettings gps_settings;
        if (mPanoGPSFiles.count(resFile) > 0)
            gps_settings = ht::PanoramaEngine::loadGPSSettings(mPanoGPSFiles[resFile]);

        auto gpsInterpolator = ht::GPSInterpolator(gps_settings, habitrack.getStartFrame());
        gpsInterpolators.push_back(gpsInterpolator);

        ht::Images currImages = habitrack.images();
        currImages.clip(habitrack.getStartFrame(), habitrack.getEndFrame() + 1);

        std::vector<cv::Point> currPts;
        if (mPanoSettings.overlayPoints)
            currPts = getDetections(habitrack);

        auto outPath = habitrack.getOutputPath() / "panorama";

        ht::setLogFileTo(outPath / "log.txt");
        ht::PanoramaEngine::runSingle(currImages, outPath, mPanoSettings, currPts,
            habitrack.getPreferences().chunkSize, gpsInterpolator);

        images.push_back(currImages);
        data.push_back(outPath);
        pts.insert(std::end(pts), std::begin(currPts), std::end(currPts));

        chunkSizes.push_back(habitrack.getPreferences().chunkSize);
    }

    if (mPanoFiles.size() <= 1)
        return;

    auto outFolder = mPanoFiles[0].parent_path() / "panorama_combined";
    ht::setLogFileTo(outFolder / "log.txt");
    for (auto resFile : mPanoFiles)
        spdlog::info("Adding res file for combined panorama: {}", resFile);
    ht::PanoramaEngine::runMulti(
        images, data, outFolder, mPanoSettings, pts, chunkSizes, gpsInterpolators);
}

std::vector<cv::Point> Tui::getDetections(const ht::HabiTrack& habitrack) const
{
    auto dets = habitrack.detections();
    auto data = dets.cdata();
    std::vector<cv::Point> vec(dets.size());
    std::transform(std::begin(data), std::end(data), std::begin(vec),
        [](auto pair) { return pair.second.position; });
    return vec;
}

void Tui::panoramaPrefs(const std::string& args)
{
    std::vector<std::string> words;
    extractWords(args, std::back_inserter(words));

    if (words.empty() || words.size() % 2 != 0)
    {
        std::cout << mPanoSettings << std::endl;
        return;
    }

    for (std::size_t i = 0; i < words.size(); i += 2)
    {
        if (words[i] == "rows")
            mPanoSettings.rows = std::stoi(words[i + 1]);
        else if (words[i] == "cols")
            mPanoSettings.cols = std::stoi(words[i + 1]);
        else if (words[i] == "cacheSize")
            mPanoSettings.cacheSize = std::stoi(words[i + 1]);
        else if (words[i] == "ftType")
        {
            if (words[i + 1] == "ORB")
                mPanoSettings.ftType = ht::FeatureType::ORB;
            else if (words[i + 1] == "SIFT")
                mPanoSettings.ftType = ht::FeatureType::SIFT;
            else if (words[i + 1] == "SuperPoint")
                mPanoSettings.ftType = ht::FeatureType::SuperPoint;
        }
        else if (words[i] == "numFts")
            mPanoSettings.numFts = std::stoi(words[i + 1]);
        else if (words[i] == "minCoverage")
            mPanoSettings.minCoverage = std::stod(words[i + 1]);
        else if (words[i] == "force")
            mPanoSettings.force = std::stoi(words[i + 1]);
        else if (words[i] == "stage")
        {
            if (words[i + 1] == "Initialization")
                mPanoSettings.stage = ht::PanoramaStage::Initialization;
            else if (words[i + 1] == "Optimization")
                mPanoSettings.stage = ht::PanoramaStage::Optimization;
            else if (words[i + 1] == "Refinement")
                mPanoSettings.stage = ht::PanoramaStage::Refinement;
        }

        else if (words[i] == "overlayCenters")
            mPanoSettings.overlayCenters = std::stoi(words[i + 1]);
        else if (words[i] == "overlayPoints")
            mPanoSettings.overlayPoints = std::stoi(words[i + 1]);
        else if (words[i] == "smooth")
            mPanoSettings.smooth = std::stoi(words[i + 1]);

        else if (words[i] == "writeReadable")
            mPanoSettings.writeReadable = std::stoi(words[i + 1]);
    }
}
} // namespace tui
