#ifndef TRACKERSCENE_H
#define TRACKERSCENE_H

#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QPixmap>

#include <QGraphicsRectItem>
#include <QInputDialog>
#include <QRect>

namespace gui
{
class TrackerScene : public QGraphicsScene
{
    Q_OBJECT
public:
    TrackerScene(QObject* parent = 0);

    void setPixmap(const QPixmap& img);

protected:
    void keyPressEvent(QKeyEvent* event);

private:
    QGraphicsPixmapItem* mImage;
};
} // namespace gui
#endif // TRACKERSCENE_H
