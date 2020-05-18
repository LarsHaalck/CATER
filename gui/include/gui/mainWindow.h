#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "gui/guiPreferences.h"
#include "gui/preferences.h"

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

private slots:
    void onSliderOverlayUnariesChanged(int value);
    void onSliderOverlayTrackedPosChanged(int value);
    void onSliderOverlayTrajectoryChanged(int value);
    void onToggleExpertMode(bool value);
    void onPreferencesClicked();

private:
    Ui::HabiTrack* ui;

    GuiPreferences mGuiPrefs;
    const GuiPreferences mGuiPrefsDefaults;

    Preferences mPrefs;

    // handles library related stuff
    // needs ui elements
    /* TrackerController mController; */
};
} // namespace gui
#endif // MAINWINDOW_H
