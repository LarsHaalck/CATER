#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThreadPool>

#include "gui/guiPreferences.h"
#include "gui/imageViewer.h"
#include "gui/progressStatusBar.h"
#include "gui/trackerScene.h"
#include "gui/unaryGraphicsView.h"
#include "habitrack/habiTrack.h"
#include <QFutureWatcher>
#include <QtConcurrent>
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
    void toggleChunk(int chunk, bool compute);

    // compute functions
    template <class Function, class Callback>
    void enqueue(Function&& func, Callback&& cb)
    {
        auto future = QtConcurrent::run(&mThreadQueue, this, func);
        auto watcher = std::make_unique<QFutureWatcher<void>>();
        watcher->setFuture(future);
        connect(watcher.get(), &QFutureWatcher<void>::finished, this, cb);
        mQueueWatchers.push(std::move(watcher));
    }

    void extractFeatures();
    void extractTrafos();
    void extractUnaries();
    void extractUnaryQualites();

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
    void on_actionLabelEditor_triggered();
    void on_actionPreferences_triggered();

    ////////////////////////////////
    // HabiTrack
    ////////////////////////////////
    void on_actionOpenImgFolder_triggered();
    void on_actionOpenImgList_triggered();
    void on_actionOpenResultsFile_triggered();
    void on_buttonStartFrame_clicked();
    void on_buttonEndFrame_clicked();
    void on_buttonTrack_clicked();
    void on_buttonExtractFeatures_clicked();
    void on_featuresExtracted();
    void on_buttonExtractTrafos_clicked();
    void on_trafosExtracted();
    void on_buttonExtractUnaries_clicked();
    void on_unariesExtracted();
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

    ////////////////////////////////
    // general
    ////////////////////////////////
    void on_actionQuit_triggered();

private:
    ////////////////////////////////
    // GUI related
    ////////////////////////////////
    Ui::MainWindow* ui;
    GuiPreferences mGuiPrefs;
    std::shared_ptr<ProgressStatusBar> mBar;
    QString mStartPath; // used for next QFileDialog
    ImageViewer mViewer;
    TrackerScene* mTrackerScene;
    UnaryScene* mUnaryScene;
    std::size_t mCurrentFrameNumber;
    bool mSaved;
    QTimer mAutoSaveTimer;
    QTimer mFrameTimer;

    ht::HabiTrack mHabiTrack;

    ////////////////////////////////
    // Threading
    ////////////////////////////////
    QMutex mMutex;
    QThreadPool mThreadQueue;
    std::queue<std::unique_ptr<QFutureWatcher<void>>> mQueueWatchers;

    std::queue<int> mDetectionsQueue;
    std::vector<double> mQualities;
    std::unordered_map<int, std::unique_ptr<QFutureWatcher<void>>> mDetectionsWatchers;
};
} // namespace gui
#endif // MAINWINDOW_H
