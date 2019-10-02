#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "gui/guiPreferences.h"

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

    bool eventFilter(QObject* obj, QEvent* event);
    void resetToDefaults(QObject* obj);

private slots:
    void onSliderOverlayUnariesChanged(int value);
    void onSliderOverlayTrackedPosChanged(int value);
    void onSliderOverlayTrajectoryChanged(int value);
    void onToggleExpertMode(bool value);
    void onPreferencesClicked();

private:
    Ui::MainWindow* ui;

    GuiPreferences mGuiPrefs;
    const GuiPreferences mGuiPrefsDefaults;

    // handles library related stuff
    // needs ui elements
    /* TrackerController mController; */
};
} // namespace gui
#endif // MAINWINDOW_H
