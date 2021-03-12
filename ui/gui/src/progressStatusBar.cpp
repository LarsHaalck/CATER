#include "gui/progressStatusBar.h"

namespace gui
{
ProgressStatusBar::ProgressStatusBar(QProgressBar* bar, QLabel* label)
    : mBar(bar)
    , mLabel(label)
{
}
} // namespace gui
