#include "gui/unaryScene.h"

#include <QLinearGradient>

#include "spdlog/spdlog.h"

constexpr double size_width = 0.7;
constexpr double size_height = 100;

namespace gui
{
UnaryScene::UnaryScene(QObject* parent)
    : QGraphicsScene(parent)
    , mNumImages(0)
    , mPen()
{
    mPen.setWidthF(size_width);
}

void UnaryScene::setTotalImages(std::size_t numImages)
{
    mNumImages = numImages;
    /* this->setSceneRect(-0.1, 0, 100.2, size_height); */
    this->setSceneRect(0, 0, 99 + size_width, size_height);
}

void UnaryScene::setFrame(std::size_t id, UnaryColor color)
{
    assert(id < mNumImages && "Unary id must be smaller than number of frames in unary color");
    auto start = ((id + size_width / 2) / mNumImages) * 100;
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
            return QColor::fromRgb(220, 150, 86); // yellow/orange
        case UnaryColor::Critical:
            return QColor::fromRgb(171, 70, 66); // red
        case UnaryColor::Good:
            return QColor::fromRgb(161, 181, 108); // green
        case UnaryColor::Undefined:
            return QColor::fromRgb(56, 56, 56); // dark gray
        default:
            return QColor::fromRgb(186, 139, 175); // magenta
    }
}
} // namespace gui
