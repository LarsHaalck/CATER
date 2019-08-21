#ifndef TRACKERGRAPHICSVIEW_H
#define TRACKERGRAPHICSVIEW_H

#include <QDebug>
#include <QGraphicsView>
#include <QMenu>
#include <QMouseEvent>
#include <QWheelEvent>

#include "gui/trackerScene.h"

namespace gui
{
class TrackerGraphicsView : public QGraphicsView
{
    Q_OBJECT

public:
    TrackerGraphicsView(QWidget* parent = 0);
    TrackerGraphicsView(TrackerScene* scene, QWidget* parent = 0);

    TrackerScene* getTrackerScene() { return this->mScene; }
    void setScale(const qreal scale);

protected:
    void wheelEvent(QWheelEvent* event);
    void mousePressEvent(QMouseEvent* event);

private:
    QString mCurrentJobName;
    TrackerScene* mScene;

signals:
    void positionChanged(QPointF position);
    void bearingChanged(QPointF position);
};
} // namespace gui
#endif // TRACKERGRAPHICSVIEW_H
