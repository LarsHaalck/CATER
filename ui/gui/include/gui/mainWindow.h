#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThreadPool>

#include "gui/guiPreferences.h"
#include "gui/imageViewer.h"
#include "gui/labelConfig.h"
#include "gui/labeler.h"
#include "gui/progressStatusBar.h"
#include "gui/trackerScene.h"
#include "gui/unaryGraphicsView.h"

#include "habitrack/habiTrack.h"
#include <QFutureWatcher>
#include <QMessageBox>
#include <QtConcurrent>
#include <deque>
#include <filesystem>
#include <unordered_set>

// needed for qRegisterMetaType
typedef std::vector<double> stdVecDouble;

namespace Ui
{
class MainWindow;
}

namespace gui
{
class MainWindow final : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

    void closeEvent(QCloseEvent* event);
    void keyPressEvent(QKeyEvent* event);

private:
    void populateGuiDefaults();
    void openImagesHelper();
    void showFrame(std::size_t frame);
    void showFrame(const cv::Mat& img);

    void updateLabels();
    void updateSlider();
    void setupProgressBar();

    bool askContinue(const QString& title, const QString& text);

    template <class Function, class Slot>
    void enqueue(Function&& func, Slot&& slot)
    {
        if (checkIfBlocked())
            return;
        mBlocked = true;
        mBackgroundThread = std::unique_ptr<QThread>(QThread::create(func, this));
        connect(mBackgroundThread.get(), &QThread::finished, this, slot);
        mBackgroundThread->start();
    }

    // compute functions
    void extractFeatures();
    void extractTrafos();
    void extractUnaries();
    void extractUnaryQualities();
    void optimizeUnaries();
    void track();
    bool checkIfBlocked(bool withOptimize = true);
    bool checkIfOptimzing();
    void loadResults();
    void enqueueOptimization();

signals:
    void toggleChunk(int chunk, bool compute);
    void warn(const QString& msg);
    void breakingBoundaryChange();

    void trafosExtracted();
    void unariesExtracted();
    void unaryQualitiesExtracted(const stdVecDouble& qualites);
    void detectionsAvailable(int chunk);
    void saveResults(bool force);

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

    void on_chunkToggled(int chunk, bool compute);

    ////////////////////////////////
    // meta
    ////////////////////////////////
    void on_actionExpertMode_toggled(bool value);
    void on_actionSave_Results_triggered();
    void on_actionLabelEditor_triggered();
    void on_actionPreferences_triggered();
    void on_warn(const QString& msg);
    void on_finished();
    void on_saveResults(bool force);

    ////////////////////////////////
    // HabiTrack
    ////////////////////////////////
    void on_actionOpenImgFolder_triggered();
    void on_actionOpenImgList_triggered();
    void on_actionOpenResultsFile_triggered();
    void on_resultsLoaded();
    void on_buttonStartFrame_clicked();
    void on_buttonEndFrame_clicked();
    void on_buttonTrack_clicked();
    void on_tracked();
    void on_buttonExtractFeatures_clicked();
    void on_buttonExtractTrafos_clicked();
    void on_trafosExtracted();
    void on_buttonExtractUnaries_clicked();
    void on_unariesExtracted();
    void on_unaryQualitiesExtracted(const stdVecDouble& qualities);
    void on_buttonOptimizeUnaries_clicked();

    void on_breakingBoundaryChange();

    ////////////////////////////////
    // HabiTrack - Interaction
    ////////////////////////////////
    void on_positionChanged(QPointF position);
    void on_bearingChanged(QPointF position);
    void on_positionCleared();
    void on_bearingCleared();
    void on_detectionsAvailable(int chunkId);

    ////////////////////////////////
    // progressbar slots
    ////////////////////////////////
    void on_totalChanged(int total);
    void on_incremented();
    void on_incremented(int inc);
    void on_isDone();
    void on_statusChanged(const QString& string);

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
    bool mLabelsSaved;
    QTimer mAutoSaveTimer;
    QElapsedTimer mFrameTimer;

    ht::HabiTrack mHabiTrack;

    ////////////////////////////////
    // Labeling
    ////////////////////////////////
    Labeler mLabeler;
    LabelGroupConfigs mLabelConfigs;

    ////////////////////////////////
    // Threading
    ////////////////////////////////
    std::deque<int> mDetectionsQueue;
    std::unordered_map<int, std::unique_ptr<QFutureWatcher<void>>> mDetectionsWatchers;
    bool mBlocked;
    std::unique_ptr<QThread> mBackgroundThread;
};
} // namespace gui
#endif // MAINWINDOW_H
