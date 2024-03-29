#ifndef PANOWIZARD_H
#define PANOWIZARD_H

#include <QFutureWatcher>
#include <QWizard>
#include <filesystem>

#include "gui/progressStatusBar.h"

#include <cater/panorama/panoramaEngine.h>
#include <cater/model/model.h>

namespace Ui
{
class PanoWizard;
}

namespace gui
{

class PanoWizard : public QWizard
{
    Q_OBJECT
public:
    explicit PanoWizard(QWidget* parent = nullptr);
    ~PanoWizard();

private:
    void process();
    ct::PanoramaSettings getSettings() const;
    std::vector<cv::Point> getDetections(const ct::Model& model) const;

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
    Ui::PanoWizard* ui;
    std::vector<std::filesystem::path> mResFiles;
    std::shared_ptr<ProgressStatusBar> mBar;
    QFutureWatcher<void> mWatcher;
};

} // namespace gui

#endif // PANOWIZARD_H
