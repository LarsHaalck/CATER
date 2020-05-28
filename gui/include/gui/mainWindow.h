#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "gui/guiPreferences.h"
#include "gui/preferences.h"
#include "gui/trackerScene.h"
#include "image-processing/features.h"
#include "image-processing/images.h"
#include "image-processing/matches.h"
#include "habitrack/unaries.h"
#include "progressbar/baseProgressBar.h"
#include <filesystem>
#include "gui/unaryGraphicsView.h"

namespace Ui
{
class HabiTrack;
}

namespace gui
{
class HabiTrack : public QMainWindow
{
    Q_OBJECT

public:
    explicit HabiTrack(QWidget* parent = nullptr);
    ~HabiTrack();

private:
    void populateGuiDefaults();
    bool eventFilter(QObject* obj, QEvent* event);
    void resetToDefaults(QObject* obj);
    void showFrame(std::size_t frame);
    void refreshWindow();
    void populatePaths(const std::filesystem::path& stem);
    void openImagesHelper();
    void setUnaryScene(std::vector<double> qualities);

    bool featureComputed() const;
    bool matchesComputed() const;
    bool unariesComputed() const;

private slots:
    void on_sliderOverlayUnaries_sliderReleased();
    void on_sliderOverlayTrackedPos_sliderReleased();
    void on_sliderOverlayTrajectory_sliderReleased();
    void on_actionExpertMode_toggled(bool value);
    void on_actionPreferences_triggered();

    void on_actionOpenImgFolder_triggered();
    void on_actionOpenImgListtriggered();
    void on_actionOpenResultsFile_triggered();

    void on_sliderFrame_sliderReleased();
    void on_spinCurrentFrame_editingFinished();
    void on_spinCurrentFrame_valueChanged(int value);
    void on_buttonPrevFrame_clicked();
    void on_buttonNextFrame_clicked();
    void on_actionPrev_Frame_triggered();
    void on_actionNext_Frame_triggered();

    void on_buttonStartFrame_clicked();
    void on_buttonEndFrame_clicked();

    void on_buttonExtractFeatures_clicked();
    void on_buttonExtractTrafos_clicked();
    void on_buttonExtractUnaries_clicked();


private:
    Ui::HabiTrack* ui;
    QString mStartPath; // used for next QFileDialog

    std::shared_ptr<ht::BaseProgressBar> mBar;

    GuiPreferences mGuiPrefs;
    const GuiPreferences mGuiPrefsDefaults;
    Preferences mPrefs;

    std::filesystem::path mOutputPath;
    std::filesystem::path mResultsFile;
    std::filesystem::path mImgFolder;
    std::filesystem::path mFtFolder;
    std::filesystem::path mMatchFolder;
    std::filesystem::path mUnFolder;
    std::filesystem::path mAntFile;

    ht::Images mImages;
    std::size_t mCurrentFrameNumber;
    std::size_t mStartFrameNumber;
    std::size_t mEndFrameNumber;
    TrackerScene* mScene;

    ht::Features mFeatures;
    ht::Unaries mUnaries;
    std::vector<double> mUnaryQualities;

    // handles library related stuff
    // needs ui elements
    /* TrackerController mController; */
};
} // namespace gui
#endif // MAINWINDOW_H
