#ifndef GUI_PROGRESS_STATUS_BAR_FUNCTOR_H
#define GUI_PROGRESS_STATUS_BAR_FUNCTOR_H

#include <habitrack/progressbar/baseProgressBar.h>
#include <QLabel>
#include <QProgressBar>
#include <string>

namespace gui
{
class ProgressStatusBar : public QObject, public ht::BaseProgressBar
{
    Q_OBJECT
public:
    ProgressStatusBar(QObject* parent);
    void setTotal(std::size_t total) { emit totalChanged(total); }
    void inc() { emit incremented(); }
    void inc(std::size_t inc) { emit incremented(inc); }
    void done() { emit isDone(); }
    void status(const std::string& state) { emit statusChanged(QString::fromStdString(state)); }
    ~ProgressStatusBar() { emit isDone(); }

signals:
    void totalChanged(int total);
    void incremented();
    void incremented(int inc);
    void isDone();
    void statusChanged(const QString& state);
};
} // namespace gui
#endif // GUI_PROGRESS_STATUS_BAR_FUNCTOR_H
