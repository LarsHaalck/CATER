#include "gui/unaryScene.h"

#include "colors.h"
#include "spdlog/spdlog.h"
#include <QLinearGradient>
#include <ostream>

constexpr double size_width = 1.0;
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
    /* this->setSceneRect(0, 0, 99 + size_width, size_height); */

    this->setSceneRect(0, 0, 100, size_height);
    mPen.setWidthF(1.0 / mNumImages * 100);

    /* this->setSceneRect(0, 0, mNumImages, size_height); */
    /* mPen.setWidthF(1.0); */

    this->clear();
    this->mUnaryColors.clear();
    this->mUnaryStates.clear();
}

void UnaryScene::setUnaryQuality(std::size_t id, UnaryQuality quality)
{
    assert(id < mNumImages && "Unary id must be smaller than number of frames in setUnaryQuality");
    mUnaryColors[id] = quality;

    auto start = idToX(id);
    auto pen = getPen(unaryQualityToQColor(mUnaryColors[id]));
    this->addLine(start, size_height / 3, start, size_height, pen);
}

void UnaryScene::setUnaryState(std::size_t id, UnaryState state)
{
    assert(id < mNumImages && "Unary id must be smaller than number of frames in setUnaryQuality");
    auto start = idToX(id);
    auto pen = getPen(unaryStateToQColor(state));
    this->addLine(start, 0, start, size_height / 3, pen);
}

void UnaryScene::update()
{
    /* for (const auto& elem : mUnaryColors) */
    /* { */
    /*     auto i = elem.first; */
    /*     auto start = idToX(i); */
    /*     auto pen = getPen(unaryQualityToQColor(mUnaryColors[i])); */
    /*     this->addLine(start, size_height / 3, start, size_height, pen); */

    /*     pen = getPen(unaryStateToQColor(mUnaryStates[i])); */
    /*     this->addLine(start, 0, start, size_height / 3, pen); */
    /* } */
}

QPen UnaryScene::getPen(const QColor& color) const
{
    auto pen = mPen;
    pen.setColor(color);
    /* pen.setWidthF(size_width); */
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
        return blue;
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
        return magenta;
    case UnaryState::DefaultAlt:
        return darkgray;
    case UnaryState::Computing:
        return lightmagenta;
    case UnaryState::ComputingAlt:
        return lightgray;
    default:
        return Qt::black;
    }
}
} // namespace gui
