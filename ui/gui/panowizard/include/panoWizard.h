#ifndef PANOWIZARD_H
#define PANOWIZARD_H

#include <QWizard>
#include <QFutureWatcher>
#include <filesystem>

#include "gui/progressStatusBar.h"
#include "panorama/panoramaEngine.h"

namespace Ui {
class PanoWizard;
}

namespace gui
{

class PanoWizard : public QWizard
{
    Q_OBJECT
public:
    explicit PanoWizard(QWidget *parent = nullptr);
    ~PanoWizard();

private:
    void process();
    ht::PanoramaSettings getSettings() const;
    void processSingle(const std::filesystem::path& resFile);
    std::vector<cv::Point> getDetections(const std::filesystem::path& resFile);
    void processMultiple();

    void populateDefaults();

private slots:
    void on_load();
    void on_newid(int id);
    void on_processEvent(int id);

    ////////////////////////////////
    // progressbar slots
    ////////////////////////////////
    void on_totalChanged(int total);
    void on_incremented();
    void on_incremented(int inc);
    void on_isDone();
    void on_statusChanged(const QString& string);

signals:
    void processEvent(int);

private:
    Ui::PanoWizard *ui;
    std::vector<std::filesystem::path> mResFiles;
    std::shared_ptr<ProgressStatusBar> mBar;
    QFutureWatcher<void> mWatcher;
};

} // namespace gui

#endif // PANOWIZARD_H
