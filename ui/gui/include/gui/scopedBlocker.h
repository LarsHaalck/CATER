#ifndef GUI_SCOPED_BLOCKER_H
#define GUI_SCOPED_BLOCKER_H

#include <QObject>
#include <spdlog/spdlog.h>

namespace gui
{
class ScopedBlocker
{
public:
    ScopedBlocker(std::initializer_list<QObject*> list)
        : mList(list)
    {
        for (auto o : mList)
        {
            if (o)
                o->blockSignals(true);
        }
    }

    ~ScopedBlocker()
    {
        for (auto o : mList)
        {
            if (o)
                o->blockSignals(false);
        }
    }
private:
    std::vector<QObject*> mList;
};
} // namespace gui

#endif // GUI_SCOPED_BLOCKER_H
