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

private slots:
    void onSliderOverlayUnariesChanged(int value);
    void onSliderOverlayTrackedPosChanged(int value);
    void onSliderOverlayTrajectoryChanged(int value);

private:
    Ui::MainWindow* ui;

    GuiPreferences mGuiPrefs;
};
} // namespace gui
#endif // MAINWINDOW_H
