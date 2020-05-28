#ifndef UNARY_SCENE_H
#define UNARY_SCENE_H

#include <QGraphicsScene>
#include <QColor>

namespace gui
{
enum class UnaryQuality
{
    Good,
    Poor,
    Critical,
    Undefined,
};

std::string unaryQualityToString(UnaryQuality quality);

class UnaryScene : public QGraphicsScene
{
    Q_OBJECT
public:
    UnaryScene(QObject* parent = 0);
    void setTotalImages(std::size_t numImages);
    std::size_t getTotalImages() const { return mNumImages; }
    void setUnaryQuality(std::size_t id, UnaryQuality color);

    UnaryQuality getUnaryQuality(std::size_t id);
    QColor getUnaryColor(std::size_t id);
private:
    QColor unaryColorToQColor(UnaryQuality quality);
private:
    std::size_t mNumImages;
    std::vector<UnaryQuality> mUnaryColors;
    QPen mPen;
};
} // namespace gui
#endif // UNARY_SCENE_H
