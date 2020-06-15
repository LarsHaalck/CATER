#include "gui/unaryScene.h"

#include "colors.h"
#include "spdlog/spdlog.h"
#include <QLinearGradient>
#include <ostream>

constexpr double size_width = 0.7;
constexpr double size_height = 100;

namespace gui
{

UnaryScene::UnaryScene(QObject* parent)
    : QGraphicsScene(parent)
    , mNumImages(0)
    , mUnaryColors()
    , mUnaryStates()
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
    this->mUnaryStates.clear();
}

void UnaryScene::setUnaryQuality(std::size_t id, UnaryQuality quality)
{
    assert(id < mNumImages && "Unary id must be smaller than number of frames in setUnaryQuality");
    mUnaryColors[id] = quality;
}

void UnaryScene::setUnaryState(std::size_t id, UnaryState state)
{
    assert(id < mNumImages && "Unary id must be smaller than number of frames in setUnaryQuality");
    mUnaryStates[id] = state;
}

void UnaryScene::update()
{
    /* for (std::size_t i = 0; i < mUnaryColors.size(); i++) */
    for (const auto& elem : mUnaryColors)
    {
        auto i = elem.first;
        auto start = idToX(i);
        auto pen = getPen(unaryQualityToQColor(mUnaryColors[i]));
        this->addLine(start, size_height / 3, start, size_height, pen);

        pen = getPen(unaryStateToQColor(mUnaryStates[i]));
        this->addLine(start, 0, start, size_height / 3, pen);
    }
}

QPen UnaryScene::getPen(const QColor& color) const
{
    auto pen = mPen;
    pen.setColor(color);
    pen.setWidthF(size_width);
    return pen;
}

double UnaryScene::idToX(std::size_t idx) const
{
    return ((idx + size_width / 2) / mNumImages) * 100;
}

UnaryQuality UnaryScene::getUnaryQuality(std::size_t id) const
{
    assert(id < mNumImages && "Unary id must be smaller than number of frames in getUnaryQuality");
    if (!mUnaryColors.count(id))
        return UnaryQuality::Undefined;
    return mUnaryColors.at(id);
}

QColor UnaryScene::getUnaryColor(std::size_t id) const
{
    return unaryQualityToQColor(getUnaryQuality(id));
}

QColor UnaryScene::unaryQualityToQColor(UnaryQuality quality) const
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
        return darkgray;
    case UnaryQuality::Excellent:
        return magenta;
    default:
        return Qt::black;
    }
}

std::string UnaryScene::unaryQualityToString(UnaryQuality quality)
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
    case UnaryQuality::Excellent:
        return "Excellent";
    default:
        return "";
    }
}

QColor UnaryScene::unaryStateToQColor(UnaryState state) const
{
    switch (state)
    {
    case UnaryState::Default:
        return blue;
    case UnaryState::DefaultAlt:
        return darkgray;
    case UnaryState::Computing:
        return cyan;
    case UnaryState::ComputingAlt:
        return lightgray;
    default:
        return Qt::black;
    }
}
} // namespace gui
