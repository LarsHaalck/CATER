#include "unaryScene.h"

#include "colors.h"
#include "spdlog/spdlog.h"

#include "image-processing/util.h"
#include "tracker/tracker.h"
#include "util/algorithm.h"
#include <QLinearGradient>
#include <ostream>

constexpr double size_width = 1.0;
constexpr double size_height = 100;

using namespace ht;

namespace gui
{

UnaryScene::UnaryScene(QObject* parent)
    : QGraphicsScene(parent)
    , mNumImages(0)
    , mUnaryColors()
    , mPen()
{
    mPen.setWidthF(size_width);
}

void UnaryScene::setTotalImages(std::size_t numImages)
{
    mNumImages = numImages;
    this->setSceneRect(0, 0, 100, size_height);
    mPen.setWidthF(1.0 / mNumImages * 100);
    this->clear();
    this->mUnaryColors.clear();
    this->mUnaryColors.reserve(numImages);
}

void UnaryScene::setUnaryQuality(std::size_t id, UnaryQuality quality)
{
    assert(id < mNumImages && "Unary id must be smaller than number of frames in setUnaryQuality");

    if (quality == UnaryQuality::Excellent)
        mUnaryBackups.insert({id, getUnaryQuality(id)});

    mUnaryColors[id] = quality;

    auto start = idToX(id);
    auto pen = getPen(unaryQualityToQColor(mUnaryColors[id]));
    this->addLine(start, size_height / 3, start, size_height, pen);
}

void UnaryScene::resetUnaryQuality(std::size_t id)
{
    if (mUnaryBackups.count(id))
    {
        setUnaryQuality(id, mUnaryBackups[id]);
        mUnaryBackups.erase(id);
    }
}

void UnaryScene::setUnaryState(std::size_t id, UnaryState state)
{
    assert(id < mNumImages && "Unary id must be smaller than number of frames in setUnaryQuality");
    auto start = idToX(id);
    auto pen = getPen(unaryStateToQColor(state));
    this->addLine(start, 0, start, size_height / 3, pen);
}

QPen UnaryScene::getPen(const QColor& color) const
{
    auto pen = mPen;
    pen.setColor(color);
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

void UnaryScene::setup(std::vector<double> qualities, std::size_t start, std::size_t end)
{
    setTotalImages(qualities.size() + 1);

    auto offset = start;
    auto num = end - start;
    std::size_t medSize = 100;
    for (std::size_t i = 0; i < std::ceil(static_cast<double>(num) / medSize); i++)
    {
        auto currStart = i * medSize + offset;
        auto currEnd = std::min((i + 1) * medSize, num) + offset;
        auto currMedian = median(
            std::vector(std::begin(qualities) + currStart, std::begin(qualities) + currEnd));

        std::vector<double> dists;
        dists.reserve(currEnd - currStart);
        for (std::size_t j = currStart; j < currEnd; j++)
            dists.push_back(std::abs(qualities[j] - currMedian));

        auto medianDist = median_fast(std::begin(dists), std::end(dists));
        for (std::size_t j = currStart; j < currEnd; j++)
        {
            UnaryQuality qual;
            if (qualities[j] == 0.0)
                qual = UnaryQuality::Critical;
            else if (qualities[j] < (currMedian + 1.5 * medianDist))
                qual = UnaryQuality::Good;
            else if (qualities[j] < (currMedian + 3.0 * medianDist))
                qual = UnaryQuality::Poor;
            else
                qual = UnaryQuality::Critical;

            setUnaryQuality(j, qual);
        }
    }
}

void UnaryScene::toggleChunk(int chunkId, bool computing, int chunkSize, int start)
{
    auto numUnaries = mUnaryColors.size();
    auto numChunks = util::getNumChunks(numUnaries, chunkSize);

    std::vector<int> chunks;
    if (chunkId == -1)
    {
        chunks.resize(numChunks);
        std::iota(std::begin(chunks), std::end(chunks), 0);
    }
    else
        chunks = {chunkId};

    for (auto chunk : chunks)
    {
        UnaryState state;
        if (computing)
            state = (chunk % 2) ? UnaryState::ComputingAlt : UnaryState::Computing;
        else
            state = (chunk % 2) ? UnaryState::DefaultAlt : UnaryState::Default;

        auto end = util::getChunkEnd(chunk, numChunks, chunkSize, numUnaries);
        for (std::size_t j = chunk * chunkSize; j < end; j++)
            setUnaryState(j + start, state);
    }
}
} // namespace gui
