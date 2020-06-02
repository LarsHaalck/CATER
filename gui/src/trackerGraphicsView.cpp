#include "gui/trackerGraphicsView.h"

namespace gui
{
TrackerGraphicsView::TrackerGraphicsView(QWidget* parent)
    : TrackerGraphicsView(new TrackerScene(parent), parent)
{
}

TrackerGraphicsView::TrackerGraphicsView(TrackerScene* scene, QWidget* parent)
    : QGraphicsView(scene, parent)
    , mScene(scene)
{
}

void TrackerGraphicsView::setScale(const qreal scale)
{
    QTransform transform = this->transform();
    this->scale(scale / transform.m11(), scale / transform.m22());
}

void TrackerGraphicsView::wheelEvent(QWheelEvent* event)
{
    if (event->modifiers().testFlag(Qt::ControlModifier))
    {
        int numDegrees = event->delta() / 8;
        int numSteps = numDegrees / 15;

        if (numSteps <= 0)
            event->ignore();

        qreal factor = 1.0 + qreal(numSteps) / 300.0;

        if (event->modifiers().testFlag(Qt::ShiftModifier))
            factor += qreal(numSteps) / 10.0;

        if (factor > 0.001)
        {
            this->scale(factor, factor);
            event->accept();
        }
        else
            event->ignore();
    }
    else
        QGraphicsView::wheelEvent(event);
}

void TrackerGraphicsView::mousePressEvent(QMouseEvent* event)
{
    if (event->modifiers().testFlag(Qt::ControlModifier)
        && !event->modifiers().testFlag(Qt::ShiftModifier)
        && event->buttons().testFlag(Qt::LeftButton))
    {
        QPointF pos = this->mapToScene(event->pos());
        emit this->positionChanged(pos);
    }
    else if (event->modifiers().testFlag(Qt::ControlModifier)
        && !event->modifiers().testFlag(Qt::ShiftModifier)
        && event->buttons().testFlag(Qt::RightButton))
    {
        emit this->positionCleared();
    }
    else if (event->modifiers().testFlag(Qt::ControlModifier)
        && event->modifiers().testFlag(Qt::ShiftModifier)
        && event->buttons().testFlag(Qt::LeftButton))
    {
        QPointF pos = this->mapToScene(event->pos());
        emit this->bearingChanged(pos);
    }
    else if (event->modifiers().testFlag(Qt::ControlModifier)
        && event->modifiers().testFlag(Qt::ShiftModifier)
        && event->buttons().testFlag(Qt::RightButton))
    {
        emit this->bearingCleared();
    }
    else
        QGraphicsView::mousePressEvent(event);
}
} // namespace gui
