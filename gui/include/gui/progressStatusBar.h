#ifndef GUI_PROGRESS_STATUS_BAR_FUNCTOR_H
#define GUI_PROGRESS_STATUS_BAR_FUNCTOR_H

#include <string>
#include <QProgressBar>
#include <QLabel>
#include "progressbar/baseProgressBar.h"

namespace gui
{
class ProgressStatusBar : public ht::BaseProgressBar
{
public:
    ProgressStatusBar(QProgressBar* bar, QLabel* label);
    void setTotal(std::size_t total) { mBar->setValue(0); mBar->setMaximum(total); }
    void inc() { mBar->setValue(mBar->value() + 1); }
    void inc(std::size_t inc) { mBar->setValue(mBar->value() + inc); }
    void done() { status("Finished"); mBar->setValue(mBar->maximum()); }
    void status(const std::string& state) { mLabel->setText(QString::fromStdString(state)); }
    ~ProgressStatusBar() { done(); }
private:
    QProgressBar* mBar;
    QLabel* mLabel;
};
} // namespace gui
#endif // GUI_PROGRESS_STATUS_BAR_FUNCTOR_H
