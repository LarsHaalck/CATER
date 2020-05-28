#include "gui/unaryScene.h"

#include "colors.h"
#include "spdlog/spdlog.h"
#include <QLinearGradient>
#include <ostream>

constexpr double size_width = 0.7;
constexpr double size_height = 100;

namespace gui
{
std::string unaryQualityToString(UnaryQuality quality)
{
    switch (quality)
    {
    case UnaryQuality::Good:
        return "Good";
    case UnaryQuality::Poor:
        return "Poor";
    case UnaryQuality::Critical:
        return "Critical";
    case UnaryQuality::Undefined:
        return "Undefined";
    default:
        return "";
    }
}

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
    this->setSceneRect(0, 0, 99 + size_width, size_height);
    this->clear();
    this->mUnaryColors.clear();

    mUnaryColors.reserve(mNumImages);
    for (std::size_t i = 0; i < numImages; i++)
        mUnaryColors.push_back(UnaryQuality::Undefined);
}

void UnaryScene::setUnaryQuality(std::size_t id, UnaryQuality quality)
{
    assert(id < mNumImages && "Unary id must be smaller than number of frames in setUnaryQuality");
    auto start = ((id + size_width / 2) / mNumImages) * 100;
    auto pen = mPen;
    pen.setColor(unaryColorToQColor(quality));
    pen.setWidthF(size_width);
    this->addLine(start, 0, start, size_height, pen);
    mUnaryColors[id] = quality;
}

UnaryQuality UnaryScene::getUnaryQuality(std::size_t id)
{
    assert(id < mNumImages && "Unary id must be smaller than number of frames in getUnaryQuality");
    return mUnaryColors[id];
}

QColor UnaryScene::getUnaryColor(std::size_t id)
{
    return unaryColorToQColor(getUnaryQuality(id));
}

QColor UnaryScene::unaryColorToQColor(UnaryQuality quality)
{
    switch (quality)
    {
    case UnaryQuality::Good:
        return green;
    case UnaryQuality::Poor:
        return orange;
    case UnaryQuality::Critical:
        return red;
    case UnaryQuality::Undefined:
        return gray;
    default:
        return magenta;
    }
}
} // namespace gui
