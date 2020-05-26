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
    this->fitInView(this->sceneRect(), Qt::IgnoreAspectRatio);
}

void UnaryGraphicsView::mousePressEvent(QMouseEvent* event)
{
    QPointF pos = this->mapToScene(event->pos());
    auto num = static_cast<std::size_t>(std::floor(pos.x()));
    spdlog::debug("GUI: mapped to unary {}", num);
    emit this->jumpedToUnary(num);
}
} // namespace gui
