#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>

#include "gui/guiPreferences.h"
#include "gui/imageViewer.h"
#include "gui/progressStatusBar.h"
#include "gui/trackerScene.h"
#include "gui/unaryGraphicsView.h"
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

    virtual void closeEvent(QCloseEvent* event);

private:
    void populateGuiDefaults();
    void openImagesHelper();
    void showFrame(std::size_t frame);
    void showFrame(const cv::Mat& img);
    void updateSlider();
    void setupProgressBar();
    bool checkRunningThread();
    void toggleChunk(int chunk, bool compute);

signals:
    void detectionsAvailable(int chunk);


private slots:
    ////////////////////////////////
    // frame vis
    ////////////////////////////////
    void on_sliderOverlayUnaries_sliderReleased();
    void on_overlayTrackedPosition_toggled(bool value);
    void on_overlayBearings_toggled(bool value);
    void on_overlayTrajectory_toggled(bool value);
    void on_trajectorySpin_valueChanged(int value);

    void on_sliderFrame_valueChanged(int value);
    void on_spinCurrentFrame_valueChanged(int value);
    void on_buttonPrevFrame_clicked();
    void on_buttonNextFrame_clicked();
    void on_actionPrev_Frame_triggered();
    void on_actionNext_Frame_triggered();

    ////////////////////////////////
    // meta
    ////////////////////////////////
    void on_actionExpertMode_toggled(bool value);
    void on_actionSave_Results_triggered();
    void on_actionLabel_Editor_triggered();
    void on_actionPreferences_triggered();

    ////////////////////////////////
    // HabiTrack
    ////////////////////////////////
    void on_actionOpenImgFolder_triggered();
    void on_actionOpenImgList_triggered();
    void on_actionOpenResultsFile_triggered();
    void on_buttonStartFrame_clicked();
    void on_buttonEndFrame_clicked();
    void on_mikeButton_clicked();
    void on_buttonExtractFeatures_clicked();
    void on_buttonExtractTrafos_clicked();
    void on_trafosExtracted();
    void on_buttonExtractUnaries_clicked();
    void on_unariesExtracted();
    void on_unariesQualitesCalculated();
    void on_buttonOptimizeUnaries_clicked();

    ////////////////////////////////
    // HabiTrack - Interaction
    ////////////////////////////////
    void onPositionChanged(QPointF position);
    void onBearingChanged(QPointF position);
    void onPositionCleared();
    void onBearingCleared();
    void onDetectionsAvailable(int chunkId);

    ////////////////////////////////
    // progressbar slots
    ////////////////////////////////
    void onTotalChanged(int total);
    void onIncremented();
    void onIncremented(int inc);
    void onIsDone();
    void onStatusChanged(const QString& string);

private:
    Ui::MainWindow* ui;

    QString mStartPath; // used for next QFileDialog
    std::shared_ptr<ProgressStatusBar> mBar;

    GuiPreferences mGuiPrefs;

    std::size_t mCurrentFrameNumber;
    TrackerScene* mScene;

    ht::HabiTrack mHabiTrack;
    ImageViewer mViewer;

    std::unique_ptr<QThread> mBlockingThread;
    std::unique_ptr<QFutureWatcher<std::vector<double>>> mUnaryQualityWatcher;
    bool mSaved;

    QMutex mMutex;
    std::queue<int> mDetectionsQueue;
    std::unordered_map<int, std::unique_ptr<QFutureWatcher<void>>> mDetectionsWatchers;
};
} // namespace gui
#endif // MAINWINDOW_H
