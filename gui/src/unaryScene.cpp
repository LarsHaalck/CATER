#include "gui/unaryScene.h"

#include <QLinearGradient>

#include "spdlog/spdlog.h"

constexpr double size_width = 0.7;
constexpr double size_height = 100;

namespace gui
{
UnaryScene::UnaryScene(QObject* parent)
    : QGraphicsScene(parent)
    , mPen()
{
    mPen.setWidthF(size_width);
}

void UnaryScene::setTotalImages(std::size_t numImages)
{
    this->setSceneRect(0, 0, numImages, size_height);
    for (std::size_t i = 0; i < numImages; i++)
        setFrame(i, UnaryColor::Good);
}

void UnaryScene::setFrame(std::size_t id, UnaryColor color)
{
    assert(id < mNumImages && "Unary id must be smaller than number of frames in unary color");
    auto start = (id + size_width / 2);
    auto pen = mPen;
    pen.setColor(unaryColorToQColor(color));
    pen.setWidthF(size_width);
    this->addLine(start, 0, start, size_height, pen);
}

QColor UnaryScene::unaryColorToQColor(UnaryColor color)
{
    switch(color)
    {
        case UnaryColor::Poor:
            return QColor::fromRgb(255, 200, 0); // yellow/orange
        case UnaryColor::Critical:
            return Qt::red; //
        case UnaryColor::Good:
            return Qt::green; //
        default:
            return QColor::fromRgb(255, 255, 255);
    }
}
} // namespace gui
