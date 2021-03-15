#ifndef GUI_VIEWMODEL_H
#define GUI_VIEWMODEL_H

#include <QApplication>
#include <memory>

namespace Ui
{
class MainWindow;
}

namespace ht
{
    class HabiTrack;
}

namespace gui
{
class ViewModel : QObject
{
    Q_OBJECT
public:
    explicit ViewModel(QObject* parent, Ui::MainWindow* ui);
    ~ViewModel();

    void setupConnections();
private:
    void setupActions();

private slots:
    void actionPreferencesTriggered();

private:
    Ui::MainWindow* mUi;
    std::unique_ptr<ht::HabiTrack> mHabiTrack;

};
} // namespace gui

#endif // GUI_VIEWMODEL_H
