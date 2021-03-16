#include "gui/imageViewer.h"

#include "image-processing/images.h"
#include "tracker/unaries.h"
#include "tracker/manualUnaries.h"
#include "tracker/detections.h"

constexpr int lH = 20;
constexpr int rH = 80;
constexpr int n = lH + rH;

constexpr int lS = 10;
constexpr int rS = 20;

namespace gui
{
ImageViewer::ImageViewer(const ht::Images& images, const ht::Unaries& unaries,
    const ht::ManualUnaries& manualUnaries, const ht::Detections& detections)
    : mImages(images)
    , mUnaries(unaries)
    , mManualUnaries(manualUnaries)
    , mDetections(detections)
    , mStart(-1)
    , mCache()
{
    mCache.resize(n);
}


cv::Mat ImageViewer::getFrame(int frameNum)
{
    /* // we only allow one background thread, otherwise it is to much book-keeping */
    /* // if it is running, joinn it first */
    /* if (mThread.joinable()) */
    /*     mThread.join(); */

    /* if (isHit(frameNum)) */
    /* { */
    /*     // just return the image, nothing else to be done */
    /*     if (isSafe(frameNum)) */
    /*         return processItem(mCache[frameNum - mStart]); */


    /*     mThread = std::thread(&ImageViewer::rebalance, this); */
    /*     // return hit */
    /* } */

    /* mThread = std::thread(&ImageViewer::buildCache, this); */
    /* return processItem(readItem(frameNum)); */
    return processItem(readItem(frameNum));
}

bool ImageViewer::isHit(int frameNum)
{
    if (mStart != -1 && (frameNum - mStart) < n)
        return true;

    return false;
}


bool ImageViewer::isSafe(int frameNum)
{
    auto id = frameNum - mStart;
    if (id < (n - rS) && id > lS)
        return true;

    return false;
}

// if thread is running, don't start another one
void ImageViewer::buildCache()
{
}

// if thread is running, don't start another one
void ImageViewer::rebalance()
{
}

ImageViewer::CacheItem ImageViewer::readItem(int frameNumber)
{
    CacheItem item;
    item.img = mImages.at(frameNumber);
    return item;
}


cv::Mat ImageViewer::processItem(const CacheItem& item)
{
    return item.img;
}

/* void showFrame(std::size_t frameNumber) */
/* { */
    /* if (frameNumber == 0) */
    /*     return; */

    /* auto idx = frameNumber - 1; */

    /* spdlog::debug("GUI: Show frame {}", idx); */
    /* mCurrentFrameNumber = frameNumber; */
    /* cv::Mat frame = mImages.at(idx); // zero indexed here */

    /* // set filename */
    /* ui->labelFileName->setText(QString::fromStdString(mImages.getFileName(idx))); */
    /* ui->labelLabel->setText(mInvisibles.count(idx) ? QString("Invisible") : QString()); */

    /* if (mUnaries.size()) */
    /* { */
    /*     auto scene = ui->unaryView->getUnaryScene(); */
    /*     auto color = scene->getUnaryColor(idx); */
    /*     auto quality = UnaryScene::unaryQualityToString(scene->getUnaryQuality(idx)); */
    /*     ui->qualityLabel->setStyleSheet( */
    /*         QString("color: white; background-color: " + color.name() + ";")); */
    /*     ui->qualityLabel->setText(quality.c_str()); */
    /* } */
    /* else */
    /* { */
    /*     ui->qualityLabel->setStyleSheet(QString("")); */
    /*     ui->qualityLabel->setText("<quality>"); */
    /* } */

    /* // overlay unary */
    /* int unarySlider = ui->sliderOverlayUnaries->value(); */
    /* if (unarySlider > 0 && mUnaries.exists(idx)) */
    /* { */
    /*     double alpha = static_cast<double>(unarySlider) / 100.0; */

    /*     cv::Mat unary; */
    /*     if (mManualUnaries.exists(idx)) */
    /*         unary = mManualUnaries.previewUnaryAt(idx); */
    /*     else */
    /*         unary = mUnaries.previewAt(idx); */

    /*     cv::Mat unaryColor; */
    /*     cv::cvtColor(unary, unaryColor, cv::COLOR_GRAY2BGR); */
    /*     std::vector<cv::Mat> channels(3); */
    /*     cv::split(unaryColor, channels); */
    /*     channels[0] = cv::Mat::zeros(unary.size(), CV_8UC1); */
    /*     cv::merge(channels, unaryColor); */

    /*     cv::resize(unaryColor, unaryColor, frame.size()); */

    /*     cv::addWeighted(unaryColor, alpha, frame, (1 - alpha), 0.0, frame); */
    /* } */

    /* // get position */
    /* if (ui->overlayTrackedPosition->isChecked() > 0 && mDetections.exists(idx)) */
    /* { */
    /*     auto detection = mDetections.at(idx); */
    /*     auto position = detection.position; */
    /*     auto theta = detection.theta; */
    /*     auto dirIndicator = rotatePointAroundPoint(position, theta); */

    /*     if (ui->overlayBearings->isChecked()) */
    /*         cv::line(frame, position, dirIndicator, cv::Scalar(0, 255, 0), 1); */

    /*     // overloay manual unary as well */
    /*     if (mManualUnaries.exists(idx)) */
    /*     { */
    /*         auto unary = mManualUnaries.previewUnaryAt(idx); */
    /*         cv::Mat unaryColor; */
    /*         cv::cvtColor(unary, unaryColor, cv::COLOR_GRAY2BGR); */
    /*         std::vector<cv::Mat> channels(3); */
    /*         cv::split(unaryColor, channels); */
    /*         channels[2] = cv::Mat::zeros(unary.size(), CV_8UC1); */
    /*         cv::merge(channels, unaryColor); */
    /*         cv::resize(unaryColor, unaryColor, frame.size()); */
    /*         cv::add(frame, unaryColor, frame); */
    /*     } */

    /*     cv::Scalar color; */
    /*     if (mInvisibles.count(idx)) */
    /*         color = cv::Scalar(100, 255, 255); */
    /*     else */
    /*         color = cv::Scalar(100, 100, 255); */

    /*     cv::circle(frame, position, 20, color, 2); */
    /* } */

    /* // get trajectory */
    /* std::size_t win = ui->trajectorySpin->value(); */
    /* if (ui->overlayTrajectory->isChecked() && win > 0 && mDetections.exists(idx)) */
    /* { */
    /*     std::size_t start = 0; */
    /*     if (idx > win) */
    /*         start = idx - win; */

    /*     auto track = mDetections.projectTo(start, idx, */
    /*         ht::matches::getTrafos(mMatchFolder, GeometricType::Homography), */
    /*         GeometricType::Homography); */
    /*     for (std::size_t i = 1; i < track.size(); ++i) */
    /*     { */
    /*         cv::Point pre = track.at(i - 1); */
    /*         cv::Point cur = track.at(i); */
    /*         cv::line(frame, pre, cur, cv::Scalar(0, 127, 255), 2); */
    /*     } */

    /*     track = mDetections.projectFrom(idx, idx + win, */
    /*         ht::matches::getTrafos(mMatchFolder, GeometricType::Homography), */
    /*         GeometricType::Homography); */
    /*     for (std::size_t i = 1; i < track.size(); ++i) */
    /*     { */
    /*         cv::Point pre = track.at(i - 1); */
    /*         cv::Point cur = track.at(i); */
    /*         cv::line(frame, pre, cur, cv::Scalar(0, 255, 255), 2); */
    /*     } */
    /* } */

    /* // set combined image and redraw */
    /* auto pixMap = QPixmap::fromImage(QtOpencvCore::img2qimgRaw(frame)); */
    /* mScene->setPixmap(pixMap); */
    /* refreshWindow(); */
/* } */


} // namespace gui
