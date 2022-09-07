#include "tui.h"

#include <cater/panorama/gpsInterpolator.h>
#include <iostream>
#include <spdlog/spdlog.h>

constexpr char prompt[] = "CATER >>> ";

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
    else if (cmd == "saveDetections")
        saveDetections(args);
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
    mModel.loadResultsFile(resFile);
    addPanorama(resFile);
}

void Tui::prefs(const std::string& args)
{
    std::vector<std::string> words;
    extractWords(args, std::back_inserter(words));

    if (words.empty() || words.size() % 2 != 0)
    {
        std::cout << mModel.getPreferences() << std::endl;
        return;
    }

    auto currPrefs = mModel.getPreferences();
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
                currPrefs.featureType = ct::FeatureType::ORB;
            else if (words[i + 1] == "SIFT")
                currPrefs.featureType = ct::FeatureType::SIFT;
            else if (words[i + 1] == "SuperPoint")
                currPrefs.featureType = ct::FeatureType::SuperPoint;
            else
            {
                std::cerr << "Unknown feature type: " << words[i + 1] << std::endl;
                std::exit(-1);
            }
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
        else if (words[i] == "ignoreManualUnaries")
            currPrefs.ignoreManualUnaries = std::stoi(words[i + 1]);
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
        else
        {
            std::cerr << "Unknown option: " << words[i] << std::endl;
            std::exit(-1);
        }
    }

    mModel.setPreferences(currPrefs);
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

    if (!fs::is_regular_file(panoFile)
        || (!gpsFile.empty()
            && ((fs::path(gpsFile).is_absolute() && !fs::is_regular_file(gpsFile))
                || !fs::is_regular_file(fs::path(panoFile).parent_path() / gpsFile))))
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
    {
        std::cout << f;
        if (mPanoGPSFiles.count(f) > 0)
            std::cout << mPanoGPSFiles[f];
        std::cout << std::endl;
    }
}

void Tui::generatePanorama()
{
    std::vector<ct::Images> images;
    std::vector<fs::path> data;

    std::vector<cv::Point> pts;
    std::vector<std::size_t> chunkSizes;
    std::vector<ct::GPSInterpolator> gpsInterpolators;
    for (auto resFile : mPanoFiles)
    {
        ct::Model cater;
        cater.loadResultsFile(resFile);
        ct::GPSSettings gps_settings;
        if (mPanoGPSFiles.count(resFile) > 0)
        {
            if (mPanoGPSFiles[resFile].is_absolute())
                gps_settings = ct::PanoramaEngine::loadGPSSettings(mPanoGPSFiles[resFile]);
            else
                gps_settings = ct::PanoramaEngine::loadGPSSettings(
                    resFile.parent_path() / mPanoGPSFiles[resFile]);
        }

        auto gpsInterpolator = ct::GPSInterpolator(gps_settings, cater.getStartFrame());
        gpsInterpolators.push_back(gpsInterpolator);

        ct::Images currImages = cater.images();
        currImages.clip(cater.getStartFrame(), cater.getEndFrame() + 1);

        std::vector<cv::Point> currPts;
        if (mPanoSettings.overlayPoints)
            currPts = getDetections(cater);

        auto outPath = cater.getOutputPath() / "panorama";

        ct::setLogFileTo(outPath / "log.txt");
        ct::PanoramaEngine::runSingle(currImages, outPath, mPanoSettings, currPts,
            cater.getPreferences().chunkSize, gpsInterpolator);

        images.push_back(currImages);
        data.push_back(outPath);
        pts.insert(std::end(pts), std::begin(currPts), std::end(currPts));

        chunkSizes.push_back(cater.getPreferences().chunkSize);
    }

    if (mPanoFiles.size() <= 1)
        return;

    auto outFolder = mPanoFiles[0].parent_path() / "panorama_combined";
    ct::setLogFileTo(outFolder / "log.txt");
    for (auto resFile : mPanoFiles)
        spdlog::info("Adding res file for combined panorama: {}", resFile);

    try
    {
        ct::PanoramaEngine::runMulti(
            images, data, outFolder, mPanoSettings, pts, chunkSizes, gpsInterpolators);
    }
    catch (const std::runtime_error& e)
    {
        spdlog::error(e.what());
    }
}

std::vector<cv::Point> Tui::getDetections(const ct::Model& cater) const
{
    auto dets = cater.detections();
    auto data = dets.cdata();
    std::vector<cv::Point> vec(dets.size());
    std::transform(std::begin(data), std::end(data), std::begin(vec),
        [](auto pair) { return pair.second.position; });
    // repeat last element to have as many detections as trafos
    vec.push_back(*vec.rbegin());
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
        else if (words[i] == "featureType")
        {
            if (words[i + 1] == "ORB")
                mPanoSettings.featureType = ct::FeatureType::ORB;
            else if (words[i + 1] == "SIFT")
                mPanoSettings.featureType = ct::FeatureType::SIFT;
            else if (words[i + 1] == "SuperPoint")
                mPanoSettings.featureType = ct::FeatureType::SuperPoint;
            else
            {
                std::cerr << "Unknown feature type: " << words[i + 1] << std::endl;
                std::exit(-1);
            }
        }
        else if (words[i] == "numFeatures")
            mPanoSettings.numFeatures = std::stoi(words[i + 1]);
        else if (words[i] == "minCoverage")
            mPanoSettings.minCoverage = std::stod(words[i + 1]);
        else if (words[i] == "force")
            mPanoSettings.force = std::stoi(words[i + 1]);
        else if (words[i] == "stage")
        {
            if (words[i + 1] == "Initialization")
                mPanoSettings.stage = ct::PanoramaStage::Initialization;
            else if (words[i + 1] == "Optimization")
                mPanoSettings.stage = ct::PanoramaStage::Optimization;
            else if (words[i + 1] == "Refinement")
                mPanoSettings.stage = ct::PanoramaStage::Refinement;
            else
            {
                std::cerr << "Unknown stage: " << words[i + 1] << std::endl;
                std::exit(-1);
            }
        }

        else if (words[i] == "overlayCenters")
            mPanoSettings.overlayCenters = std::stoi(words[i + 1]);
        else if (words[i] == "overlayPoints")
            mPanoSettings.overlayPoints = std::stoi(words[i + 1]);
        else if (words[i] == "smooth")
            mPanoSettings.smooth = std::stoi(words[i + 1]);

        else if (words[i] == "writeReadable")
            mPanoSettings.writeReadable = std::stoi(words[i + 1]);
        else
        {
            std::cerr << "Unknown option: " << words[i + 1] << std::endl;
            std::exit(-1);
        }
    }
}
} // namespace tui
