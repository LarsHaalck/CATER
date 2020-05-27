#include "gui/unaryGraphicsView.h"

#include "spdlog/spdlog.h"
#include <QMouseEvent>

namespace gui
{
UnaryGraphicsView::UnaryGraphicsView(QWidget* parent)
    : UnaryGraphicsView(new UnaryScene(parent), parent)
{
}

UnaryGraphicsView::UnaryGraphicsView(UnaryScene* scene, QWidget* parent)
    : QGraphicsView(scene, parent)
    , mScene(scene)
{
}

void UnaryGraphicsView::resizeEvent(QResizeEvent*)
{
    this->fitInView(mScene->sceneRect(), Qt::IgnoreAspectRatio);
}

void UnaryGraphicsView::mousePressEvent(QMouseEvent* event)
{
    auto numImgs = this->mScene->getTotalImages();
    if (numImgs == 0)
        return;

    QPointF pos = this->mapToScene(event->pos());
    auto num = static_cast<int>(std::floor((pos.x() / 100) * numImgs));
    spdlog::debug("GUI: mapped to unary {}", num);
    if (num < 0)
        return
    emit this->jumpedToUnary(num);
}
} // namespace gui
