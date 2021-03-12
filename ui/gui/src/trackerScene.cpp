#include "gui/trackerScene.h"

namespace gui
{
TrackerScene::TrackerScene(QObject* parent)
    : QGraphicsScene(parent)
    , mImage(addPixmap(QPixmap()))
{
}

void TrackerScene::setPixmap(const QPixmap& img) { this->mImage->setPixmap(img); }

void TrackerScene::keyPressEvent(QKeyEvent* event) { QGraphicsScene::keyPressEvent(event); }
} // namespace gui
