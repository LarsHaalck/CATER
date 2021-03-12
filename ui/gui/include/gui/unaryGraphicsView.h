#ifndef UNARY_GRAPHICSVIEW_H
#define UNARY_GRAPHICSVIEW_H

#include <QGraphicsView>

#include "gui/unaryScene.h"

namespace gui
{
class UnaryGraphicsView : public QGraphicsView
{
    Q_OBJECT

public:
    UnaryGraphicsView(QWidget* parent = 0);
    UnaryGraphicsView(UnaryScene* scene, QWidget* parent);
    UnaryScene* getUnaryScene() { return this->mScene; }

protected:
    void resizeEvent(QResizeEvent* event);
    void mousePressEvent(QMouseEvent* event);
signals:
    void jumpedToUnary(std::size_t frameNumber);

private:
    UnaryScene* mScene;
    std::size_t mNumFrames;
};
} // namespace gui
#endif // UNARY_GRAPHICSVIEW_H
