#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>

#include "gui/guiPreferences.h"
#include "gui/trackerScene.h"
#include "gui/unaryGraphicsView.h"
#include "gui/imageViewer.h"
#include "progressbar/baseProgressBar.h"
#include "habitrack/habiTrack.h"
#include <QFutureWatcher>
#include <deque>
#include <filesystem>
#include <unordered_set>


namespace Ui
{
class MainWindow;
}



namespace gui
{
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private:
    void populateGuiDefaults();
    void openImagesHelper();

    void showFrame(std::size_t frame);
    void showFrame(const cv::Mat& img);

    void refreshWindow();
    void populatePaths();
    void setupUnaryScene(std::vector<double> qualities);
    void toggleChunkUnaryScene(int chunk, bool computing);

private slots:
    void on_sliderOverlayUnaries_sliderReleased();
    void on_overlayTrackedPosition_toggled(bool value);
    void on_overlayBearings_toggled(bool value);
    void on_overlayTrajectory_toggled(bool value);
    void on_trajectorySpin_valueChanged(int value);

    void on_actionExpertMode_toggled(bool value);
    void on_actionSave_Results_triggered();
    void on_actionLabel_Editor_triggered();
    void on_actionPreferences_triggered();

    void on_actionOpenImgFolder_triggered();
    void on_actionOpenImgList_triggered();
    void on_actionOpenResultsFile_triggered();

    void on_sliderFrame_valueChanged(int value);
    void on_spinCurrentFrame_valueChanged(int value);
    void on_buttonPrevFrame_clicked();
    void on_buttonNextFrame_clicked();
    void on_actionPrev_Frame_triggered();
    void on_actionNext_Frame_triggered();

    void on_buttonStartFrame_clicked();
    void on_buttonEndFrame_clicked();

    void on_mikeButton_clicked();
    void on_buttonExtractFeatures_clicked();
    void on_buttonExtractTrafos_clicked();
    void on_buttonExtractUnaries_clicked();
    void on_buttonOptimizeUnaries_clicked();

    void onPositionChanged(QPointF position);
    void onBearingChanged(QPointF position);
    void onPositionCleared();
    void onBearingCleared();

    void onDetectionsAvailable(int chunkId);

    /* void onBlockingThreadFinished(); */

private:
    Ui::MainWindow* ui;

    QString mStartPath; // used for next QFileDialog
    std::shared_ptr<ht::BaseProgressBar> mBar;

    GuiPreferences mGuiPrefs;
    /* model::Preferences mPrefs; */

    /* std::filesystem::path mOutputPath; */
    /* std::filesystem::path mResultsFile; */
    /* std::filesystem::path mImgFolder; */
    /* std::filesystem::path mFtFolder; */
    /* std::filesystem::path mMatchFolder; */
    /* std::filesystem::path mUnFolder; */
    /* std::filesystem::path mDetectionsFile; */
    /* std::filesystem::path mSetFile; */

    /* ht::Images mImages; */
    std::size_t mCurrentFrameNumber;
    TrackerScene* mScene;

    /* ht::Features mFeatures; */
    /* ht::Unaries mUnaries; */
    /* ht::ManualUnaries mManualUnaries; */
    /* std::vector<double> mUnaryQualities; */
    /* std::unordered_map<std::size_t, UnaryQuality> mUnaryQualityValues; */
    /* std::unordered_set<std::size_t> mInvisibles; */

    /* ht::Detections mDetections; */
    /* std::unordered_map<int, std::unique_ptr<QFutureWatcher<ht::Detections>>> mDetectionsWatchers; */
    /* std::deque<int> mDetectionsQueue; */
    /* QMutex mMutex; */

    ht::HabiTrack mHabiTrack;
    ImageViewer mViewer;

    double total;
    int num;

    /* std::filesystem::path mImgFolder; */
    /* QThread* mBlockingThread; */
};
} // namespace gui
#endif // MAINWINDOW_H
